import itertools
import json
import os
from enum import Enum

import virxrlcu
from rlbot.utils.structures.quick_chats import QuickChats

# don't import like this...
from util.agent import *
from util.routines import *
from util.tools import find_any_shot, find_any_shot_towards, find_shot
from util.utils import *


class Playstyle(Enum):
    Defensive = -1
    Neutral = 0
    Offensive = 1


class VirxEB(VirxERLU):
    def __init__(self, name, team, index):
        super().__init__(name, team, index)

        self.debug_2d_bool = self.name == "VirxEB"
        self.debug_profiles = False
        self.debug_vector = Vector()
        self.goalie = False
        self.jump = True
        self.double_jump = True
        self.ground_shot = True
        self.aerials = True

    def initialize_agent(self):
        super().initialize_agent()

        self.cheating = self.true_name == "VirxEMB"
        if self.cheating:
            self.boost_amount = "unlimited"

        self.playstyles = Playstyle
        self.playstyle = self.playstyles.Neutral

        self.predictions = {
            "closest_enemy": 0,
            "team_from_goal": (),
            "team_to_ball": (),
            "self_from_goal": 0,
            "self_to_ball": 0,
            "was_down": False
        }

        self.closest_foes = []
        self.friend_times = []

        self.profiler_threshold = 0.8
        self.profiler_loss = 0.005
        self.profiler_gain = 0.21
        self.profiler_last_save = 0
        self.last_ball_touch_time = -1
        self.unpause_timer = -1

        self.enemy_time_to_ball = 7
        self.min_intercept_slice = 180
        self.own_goal = {"location": Vector(), "slice": -1}
        self.is_own_goal = False
        self.is_goal = False

    def init(self):
        foe_team = -1 if self.team == 1 else 1
        team = -foe_team

        # note that the second value may be positive or negative
        self.kickoff_left = (-2048 * self.friend_team_side, 2560)
        self.kickoff_right = (2048 * self.friend_team_side, 2560)
        self.kickoff_back = (0, 4608)
        self.kickoff_back_left = (-256 * self.friend_team_side, 3840)
        self.kickoff_back_right = (256 * self.friend_team_side, 3840)

        self.best_shot = (Vector(foe_team * 793, foe_team * 5213, min(self.ball_radius * 3, 321.3875)), Vector(-foe_team * 793, foe_team * 5213, max(642.775 - self.ball_radius * 2, 321.3875)))
        self.offensive_shot = (Vector(foe_team * 893, foe_team * 5120, min(self.ball_radius * 2, 321.3875)), Vector(foe_team * -893, foe_team * 5120, max(642.775 - self.ball_radius, 321.3875)))
        self.anti_shot = (Vector(-team * 1536, team * 5120, 1285.55), Vector(team * 1536, team * 5120, 1285.55))

        self.shots = (
            (
                self.best_shot,
                False
            ),
            (
                self.offensive_shot,
                False
            ),
            (
                self.best_shot,
                True
            ),
            (
                self.offensive_shot,
                True
            )
        )

        self.max_shot_weight = self.get_weight(self.best_shot)

        if self.name in {"VirxEB", "VirxEMB"}:
            print(f"{self.name}: Check me out at https://www.virxcase.dev!!!")

    def save_profiles(self):
        for car in self.friends + self.foes:
            with open(car.profile_path, "w") as f:
                json.dump(car.profile, f)

        self.profiler_last_save = self.time

    def refresh_player_lists(self, packet: GameTickPacket):
        if not os.path.isdir(os.path.dirname(__file__) + '/../Vfiles'):
            self.print("No profiles found, creating directory for them")
            os.mkdir(os.path.dirname(__file__) + '/../Vfiles')

        # Useful to keep separate from get_ready because humans can join/leave a match
        self.friends = tuple(car_object(i, packet) for i in range(packet.num_cars) if packet.game_cars[i].team is self.team and i != self.index)
        self.foes = tuple(car_object(i, packet) for i in range(packet.num_cars) if packet.game_cars[i].team != self.team)
        self.me = car_object(self.index, packet)
        self.true_name = self.me.true_name

        len_friends = str(len(self.friends))
        self.me.profile[len_friends] = [1, 1, 1, 1]

        for car in self.friends:
            if car.profile.get(len_friends) is None:
                car.profile[len_friends] = [1, 1, 1, 1]

        len_foes = str(len(self.foes) - 1)
        for car in self.foes:
            if car.profile.get(len_foes) is None:
                car.profile[len_foes] = [1, 1, 1, 1]

    def preprocess(self, packet):
        super().preprocess(packet)

        # number of not demolished friends and foes
        self.alive_friends = []
        self.num_friends = 0
        self.alive_foes = []
        self.num_foes = 0

        for f in self.friends:
            if not f.demolished:
                self.num_friends += 1
                self.alive_friends.append(f)
        
        for f in self.foes:
            if not f.demolished:
                self.num_foes += 1
                self.alive_foes.append(f)

        if self.delta_time > 0 and self.game.round_active and self.time - self.unpause_timer > 4 and len(self.foes) > 0:
            # adjust the profiles of each bot
            loss = self.profiler_loss * self.delta_time
            lens = (str(len(self.friends)), str(len(self.foes) - 1))

            dbz = self.ball.location.z
            divisors = [
                dbz <= 126.75,
                126.75 < dbz <= 312.75,
                312.75 < dbz <= 542.75,
                542.75 < dbz
            ]
            section = divisors.index(True)

            for car in self.friends:
                if not car.demolished:
                    car.profile[lens[0]][section] = max(car.profile[lens[0]][section] - (loss / (len(self.friends) + 1)), 0)

            for car in self.foes:
                if not car.demolished:
                    car.profile[lens[1]][section] = max(car.profile[lens[1]][section] - (loss / len(self.foes)), 0)

            gain = self.profiler_gain + loss

            if self.ball.last_touch.time != self.last_ball_touch_time:
                if self.ball.last_touch.car.index != self.index:
                    if self.ball.last_touch.car.team == self.team:
                        index = None
                        for i, car in enumerate(self.friends):
                            if car.index == self.ball.last_touch.car.index:
                                index = i

                        self.friends[index].profile[lens[0]][0 if not car.airborne else section] = min(self.friends[index].profile[lens[0]][section] + gain, 1)
                    else:
                        index = None
                        for i, car in enumerate(self.foes):
                            if car.index == self.ball.last_touch.car.index:
                                index = i

                        self.foes[index].profile[lens[1]][0 if not car.airborne else section] = min(self.foes[index].profile[lens[1]][section] + gain, 1)
                self.last_ball_touch_time = self.ball.last_touch.time

        if self.name == "VirxEB" and self.time - self.profiler_last_save > 10:
            self.save_profiles()

    def get_weight(self, shot, towards=False):
        min_shot_weight = -1 if towards else 2
        
        if shot is self.anti_shot:
            return min_shot_weight + 1

        if shot is self.offensive_shot:
            return min_shot_weight + 2

        if shot is self.best_shot:
            return min_shot_weight + 3

        return 0

    def update_predictions(self):
        is_other_tick = self.odd_tick % 2 == 0
        is_forth_tick = self.odd_tick == 0
        if self.num_foes > 0:
            foe_distances = tuple(self.ball.location.flat_dist(foe.location) for foe in self.alive_foes)
            if len(foe_distances) > 0:
                if is_forth_tick or len(self.closest_foes) != self.num_foes:
                    for foe in self.foes:
                        foe.minimum_time_to_ball = self.time_to_ball(foe)

                    self.closest_foes = sorted([foe for foe in self.foes], key=lambda foe: foe.minimum_time_to_ball)
                    self.enemy_time_to_ball = self.closest_foes[0].minimum_time_to_ball

                self.predictions['closest_enemy'] = min(foe_distances)
            else:
                self.closest_foes = []
                self.enemy_time_to_ball = 7
                self.predictions['closest_enemy'] = math.inf
        else:
            self.closest_foes = []
            self.enemy_time_to_ball = 7
            self.predictions['closest_enemy'] = math.inf

        self.future_ball_location_slice = min(round(self.enemy_time_to_ball * 60), self.ball_prediction_struct.num_slices - 1)
        self.dbg_2d(f"Predicted enemy time to ball: {round(self.enemy_time_to_ball, 1)}")

        self.predictions['self_from_goal'] = self.friend_goal.location.flat_dist(self.me.location)
        self.predictions['self_to_ball'] = self.ball.location.flat_dist(self.me.location)

        if not self.predictions['was_down']:
            self.predictions['was_down'] = self.game.friend_score - self.game.foe_score > 1

        if self.num_friends > 0:
            teammates = tuple(itertools.chain(self.alive_friends, [self.me]))

            self.predictions["team_from_goal"] = sorted(tuple(self.friend_goal.location.flat_dist(teammate.location) for teammate in teammates))
            self.predictions["team_to_ball"] = sorted(tuple(self.ball.location.flat_dist(teammate.location) for teammate in teammates))

        if self.cheating:
            if is_other_tick:
                cars = { self.index: CarState(boost_amount=100) }
                self.set_game_state(GameState(cars=cars))
            self.me.boost = 100

        if is_forth_tick:
            self.me.minimum_time_to_ball = self.time_to_ball(self.me)
        self.min_intercept_slice = min(round(self.me.minimum_time_to_ball * 60), self.ball_prediction_struct.num_slices - 1)
        self.dbg_2d(f"Minimum time to ball: {round(self.me.minimum_time_to_ball, 1)}")

        for friend in self.friends:
            if friend.demolished:
                friend.minimum_time_to_ball = 7
                continue

            action = friend.tmcp_action
            if action is not None:
                action_type = action['type']

                if action_type in {"BALL", "READY"} and action['time'] != -1:
                    time = action['time'] - self.time
                    friend.minimum_time_to_ball = time if time > 0 else 7
                else:
                    friend.minimum_time_to_ball = 7
            elif is_forth_tick:
                mttb = self.time_to_ball(friend) * abs(friend.local_velocity().angle2D(friend.local_location(self.ball.location)))
                if mttb > 6: mttb = 7
                friend.minimum_time_to_ball = mttb

        self.friend_times = sorted([friend for friend in self.alive_friends], key=lambda friend: friend.minimum_time_to_ball)

        if is_other_tick:
            is_own_goal = False
            is_goal = False

            if self.ball_prediction_struct is not None:
                own_goal_slice = -1

                for i, ball_slice in enumerate(self.ball_prediction_struct.slices[30::12]):
                    location = ball_slice.physics.location.y * self.friend_team_side

                    if location >= 5212.75:
                        is_own_goal = True
                        own_goal_slice = i
                        break

                    if location <= -5212.75:
                        is_goal = True
                        break

                self.own_goal = {"location": Vector(), "slice": -1}

                if is_own_goal:
                    for i, ball_slice in enumerate(self.ball_prediction_struct.slices[::15]):
                        if ball_slice.physics.location.y * self.friend_team_side >= 5200:
                            self.own_goal["location"] = Vector.from_vector(ball_slice.physics.location)
                            self.own_goal["slice"] = i
                            break

                    if self.cheating and not self.shooting:
                        tp_own_goal = None
                        for ball_slice in self.ball_prediction_struct.slices[:30]:
                            if ball_slice.physics.location.y * self.friend_team_side >= 5200:
                                tp_own_goal = ball_slice.physics.location
                                break

                        if tp_own_goal is not None:
                            cars = { self.index: CarState(Physics(location=Vector3(tp_own_goal.x, tp_own_goal.y, tp_own_goal.z), velocity=Vector3(0, 0, 0))) }
                            self.set_game_state(GameState(cars=cars))

                if is_own_goal and not self.is_own_goal:
                    self.send_quick_chat(QuickChats.CHAT_EVERYONE, QuickChats.Compliments_NiceShot)

                if is_goal and not self.is_goal:
                    self.send_quick_chat(QuickChats.CHAT_EVERYONE, QuickChats.Reactions_Wow)

            self.is_own_goal = is_own_goal
            self.is_goal = is_goal

    def test(self):
        # shot accuracy testing
        ""

        ""
        # slope testing
        """
        ball_state = BallState(Physics(location=Vector3(*self.debug_vector), velocity=Vector3(0, 0, 0), angular_velocity=Vector3(0, 0, 0)))
        self.set_game_state(GameState(ball=ball_state))

        car_to_ball = self.ball.location - self.me.location
        direction = car_to_ball.normalize()
        shot_vector = direction.clamp((self.foe_goal.left_post - self.ball.location).normalize(), (self.foe_goal.right_post - self.ball.location).normalize())
        car_to_ball = car_to_ball.flatten()
        shot_vector = shot_vector.flatten()

        d = shot_vector.dot(car_to_ball)
        e = abs(shot_vector.cross((0, 0, 1)).dot(car_to_ball))

        slope = 10 * sign(d) if (e == 0) else max(min(d / e, 3), -3)
        self.dbg_2d(f"Slope: {slope}")

        shot = self.get_shot(self.offensive_shots[0], cap=6)
        if shot is not None:
            self.dbg_2d(f"Time: {shot['intercept_time'] - self.time}")
        else:
            self.dbg_2d(f"Time: impossible")
        """
        # Enemy intercept prediction testing
        """
        if self.enemy_time_to_ball != 7:
            intercept_slice = self.future_ball_location_slice
            intercept_location = self.ball_prediction_struct.slices[intercept_slice].physics.location
            intercept_location = Vector(intercept_location.x, intercept_location.y, intercept_location.z)
            self.sphere(intercept_location, self.ball_radius, self.renderer.black())
        """
        # Backcheck testing
        """
        self.dbg_2d(round(self.ball.location))

        if self.is_clear():
            self.backcheck()
        """
        # Shot testing
        """
        if (not self.shooting or self.odd_tick == 0) and self.smart_shot(self.best_shot, cap=6):
            return

        if not self.shooting:
            if self.is_clear() and not self.me.airborne and (self.me.boost < 60 or not self.smart_shot(self.best_shot, cap=6)) and self.me.location.flat_dist(self.debug_vector * self.friend_team_side) > 250:
                self.push(face_target(ball=True))
                self.push(goto(self.debug_vector.flatten() * self.friend_team_side, brake=True))

            if self.ball.location.z < 150:  # and not self.is_goal:
                ball_state = BallState(Physics(location=Vector3(0, -4096 * self.friend_team_side, self.ball.location.z), velocity=Vector3(0, 0, 2000), angular_velocity=Vector3(0, 0, 0)))
                # ball_state = BallState(Physics(location=Vector3(0, 0, self.ball.location.z), velocity=Vector3(0, 0, 2000), angular_velocity=Vector3(0, 0, 0)))
                self.set_game_state(GameState(ball=ball_state))
        """
        # Recovery testing
        """
        if not self.me.airborne and self.me.velocity.magnitude() < 10:
            cars = {
                self.index: CarState(Physics(location=Vector3(0, 0, 17.1), velocity=Vector3(*self.debug_vector), angular_velocity=Vector3(0, 0, 0)))
            }
            self.set_game_state(GameState(cars=cars))
        elif self.me.airborne and self.is_clear():
            self.push(ball_recovery())
        """
        # Ceiling aerial testing
        """
        if not self.shooting and self.ball.location.z < 100 and not self.is_goal:
            ball_state = BallState(Physics(location=Vector3(self.debug_vector.x * self.friend_team_side, self.debug_vector.y * self.friend_team_side, self.ball.location.z), velocity=Vector3(0, 0, 2000), angular_velocity=Vector3(0, 0, 0)))
            game_state = GameState(ball=ball_state)
            self.set_game_state(game_state)

        if self.is_clear():
            self.push(ceiling_shot())
        """

    def time_to_ball(self, car):
        if car.demolished:
            return 7

        # Assemble data in a form that can be passed to C
        me = car.get_raw(self)

        g_me = car.get_raw(self, car.location.z < 300)
        is_on_ground = g_me[5] == 0

        profile = [profile > self.profiler_threshold for profile in car.profile[str(len(self.friends)) if car.team == self.team else str(len(self.foes) - 1)]]

        game_info = (
            self.boost_accel,
            self.ball_radius
        )

        gravity = tuple(self.gravity)

        start_slice = cap(round(car.minimum_time_to_ball * 60) - 60, 12, self.ball_prediction_struct.num_slices - 1) if car.minimum_time_to_ball != 7 else 12
        end_slice = cap(round(car.minimum_time_to_ball * 60) + 60, 12, self.ball_prediction_struct.num_slices - 1)

        # check every 12th slice
        for ball_slice in self.ball_prediction_struct.slices[start_slice:end_slice:5]:
            # Gather some data about the slice
            time_remaining = ball_slice.game_seconds - self.time

            if time_remaining <= 0:
                return 7

            ball_location = (ball_slice.physics.location.x, ball_slice.physics.location.y, ball_slice.physics.location.z)

            if abs(ball_location[1]) > 5212.75:
                return 7  # abandon search if ball is scored at/after this point

            ball_info = (ball_location, (ball_slice.physics.velocity.x, ball_slice.physics.velocity.y, ball_slice.physics.velocity.z))

            # Check if we can make a shot at this slice
            # This operation is very expensive, so we use C to improve run time
            # We also need to separate ground shots from aerials shots
            shot = virxrlcu.parse_slice_for_shot(is_on_ground and profile[0], is_on_ground and profile[1], is_on_ground and profile[2], 0, time_remaining, *game_info, gravity, ball_info, g_me)

            if shot['found'] == 1:
                return time_remaining
            else:
                shot = virxrlcu.parse_slice_for_shot(0, 0, 0, profile[3] and ball_location[2] > 300, time_remaining, *game_info, gravity, ball_info, me, me[5])

                if shot['found'] == 1:
                    return time_remaining

        return 7

    def run(self):
        # predictions
        self.update_predictions()

        self.test()
        return

        # act on the predictions
        if not self.kickoff_done:
            if self.is_clear():
                self.kickoff()
            return

        self.render_time_predictions()

        friend_time_to_ball = 6 if self.num_friends > 0 else 4
        any_going_for_ball = False

        for friend in self.friends:
            action = friend.tmcp_action
            if action is not None and action["type"] == "BALL" and action['time'] != -1:
                time = action['time'] - self.time + 0.5
                if time > 0:
                    any_going_for_ball = True
                    friend_time_to_ball = min(friend_time_to_ball, time)

        is_closest_to_ball = self.num_friends == 0 or self.me.minimum_time_to_ball < self.friend_times[0].minimum_time_to_ball + 0.5

        if self.goalie:
            self.playstyle = self.playstyles.Defensive
        elif self.num_friends == 0:
            self.playstyle = self.playstyles.Neutral if self.num_foes <= 1 or self.cheating else self.playstyles.Offensive
        elif self.num_friends == 1 or self.is_own_goal:
            if is_closest_to_ball:
                self.playstyle = self.playstyles.Offensive
            else:
                self.playstyle = self.playstyles.Neutral if self.num_foes <= 2 or self.cheating else self.playstyles.Defensive
        elif self.num_friends >= 2:
            if is_closest_to_ball:
                self.playstyle = self.playstyles.Offensive
            else:
                retreat_target = retreat().get_target(self)
                self.playstyle = self.playstyles.Defensive if min(friend.location.flat_dist(retreat_target) + friend.location.z for friend in self.friends) > self.me.location.flat_dist(retreat_target) + self.me.location.z else self.playstyles.Neutral

        self.dbg_3d(self.playstyle.name)

        if self.game_mode == "heatseeker":
            if not self.is_own_goal:
                if self.get_stack_name() == "ball_recovery":
                    self.clear()

                if self.is_clear():
                    self.push(Hover(self.friend_goal.location))

                return

            if self.get_stack_name() in {"Hover", None} and self.smart_shot(cap=6):
                return

            if self.is_clear():
                self.push(Hover(self.own_goal["location"]))
                return

            return

        any_ready = any((f.tmcp_action is not None and f.tmcp_action['type'] == "READY" and (0 < f.tmcp_action['time'] - self.time < 4) for f in self.friends))
        current_shot_weight = self.stack[0].weight if self.shooting else -1

        if (not self.is_own_goal and self.ball.location.dist(self.friend_goal.location) > 1920) and self.shooting and self.num_friends > 0 and current_shot_weight < self.max_shot_weight:
            self.tmcp_ball_check()

        if self.is_own_goal or self.get_stack_name() == "goto_boost":
            self.boost_viability_check()

        if not self.is_own_goal and self.num_friends > 0 and (self.playstyle is not self.playstyles.Offensive or any_ready):
            if not self.me.airborne and self.me.boost < 36 and self.boost_amount != "no boost":
                if self.is_clear():
                    self.goto_nearest_boost()

                if not self.is_clear():
                    return

        if not self.is_clear() and self.odd_tick != 0:
            return

        if not self.is_own_goal and (is_closest_to_ball or self.playstyle is not self.playstyles.Defensive):
            for shot in self.shots:
                shot_weight = self.get_weight(*shot)

                if self.shooting and shot_weight < current_shot_weight:
                    break

                if self.smart_shot(shot[0], shot_weight, 5, shot[1]):
                    return

        anti_shot_weight = self.get_weight(self.anti_shot)
        if current_shot_weight <= anti_shot_weight and (self.is_own_goal or is_closest_to_ball or not any_going_for_ball or not any_ready) and not (self.cheating and self.is_own_goal):
            if self.smart_shot(self.anti_shot, anti_shot_weight, friend_time_to_ball):
                return

        if self.me.airborne:
            self.air_recovery()
            return

        if not self.is_own_goal and self.boost_amount != "no boost" and ((self.num_friends == 0 and self.me.boost < 36) or (self.num_friends > 1 and self.me.boost < 72)):
            if self.is_clear():
                self.goto_nearest_boost()

            if not self.is_clear():
                return

        if self.is_clear():
            self.backcheck()

    def get_minimum_game_time_to_ball(self):
        return self.me.minimum_time_to_ball + self.time

    def handle_match_comm(self, msg):
        if not self.kickoff_done and msg.get("VirxERLU") is not None and msg['VirxERLU']['team'] is self.team:
            msg = msg['VirxERLU']
            if msg.get("attacking", False) and msg['index'] < self.index and (self.is_clear() or self.get_stack_name() in {"corner_kickoff", "back_offset_kickoff", "back_kickoff", "generic_kickoff"}):
                self.clear()
                self.defensive_kickoff()

    def kickoff(self):
        if len(self.friends) > 0:
            if almost_equals(self.predictions['team_to_ball'][0], self.predictions['self_to_ball'], 5) and (self.predictions['self_to_ball'] < self.predictions['closest_enemy'] or almost_equals(self.predictions['closest_enemy'], self.predictions['self_to_ball'], 10)):
                self.offensive_kickoff()
            elif almost_equals(self.predictions['team_to_ball'][-1], self.predictions['self_to_ball'], 5):
                self.defensive_kickoff()
        elif len(self.foes) == 0 or self.predictions['self_to_ball'] < self.predictions['closest_enemy'] or almost_equals(self.predictions['closest_enemy'], self.predictions['self_to_ball'], 10):
            self.offensive_kickoff()
        else:
            self.defensive_kickoff()

    def render_time_predictions(self):
        enemy_intercept_location = self.ball_prediction_struct.slices[self.future_ball_location_slice].physics.location
        enemy_intercept_location = Vector(enemy_intercept_location.x, enemy_intercept_location.y, enemy_intercept_location.z)
        if self.enemy_time_to_ball != 7:
            self.sphere(enemy_intercept_location, self.ball_radius, self.renderer.red())

        self_intercept_location = self.ball_prediction_struct.slices[self.min_intercept_slice].physics.location
        self_intercept_location = Vector(self_intercept_location.x, self_intercept_location.y, self_intercept_location.z)
        if self.me.minimum_time_to_ball != 7:
            self.sphere(self_intercept_location, self.ball_radius, self.renderer.green())

        cap_location = self.ball_prediction_struct.slices[round(get_cap(self, 6) * 60) - 1].physics.location
        cap_location = Vector(cap_location.x, cap_location.y, cap_location.z)
        self.sphere(cap_location, self.ball_radius, self.renderer.orange())

    def air_recovery(self):
        if (self.is_clear() or self.get_stack_name() == "ball_recovery") and self.gravity.z > -700 and self.me.location.z > 750 and self.predictions['self_to_ball'] > 2560:
            if not self.is_clear():
                self.clear()

            self.push(boost_down())
            return

        if self.is_clear():
            self.push(recovery())

    def tmcp_ball_check(self):
        if self.playstyle is self.playstyles.Neutral:
            return

        shot_name = self.get_stack_name()
        shot_time = self.stack[0].intercept_time if shot_name != "short_shot" else 7
        # self_before_foe = shot_time < self.enemy_time_to_ball + 0.5
        shot_vector_angle = self.stack[0].shot_vector.angle(self.foe_goal.location - self.me.location)

        if shot_time > 1 and (shot_name != "Aerial" or shot_time > 3):
            is_short_shot = shot_name == "short_shot"
            for friend in self.friends:
                action = friend.tmcp_action
                if action is not None and action['type'] == "BALL" and action['time'] != -1:
                    f_shot_vector = Vector(*action['direction'])
                    dir_is_unknown = f_shot_vector.magnitude() == 0
                    if is_short_shot and not dir_is_unknown:
                        self.clear()
                        break

                    action_time = action['time'] - self.time
                    if action_time > 0 and f_shot_vector.angle(self.foe_goal.location - friend.location) < shot_vector_angle - 0.05:
                        time_dif = 0.2 if dir_is_unknown else 0.5
                        if (action_time < shot_time - time_dif and shot_time > self.enemy_time_to_ball + time_dif) or (action_time > shot_time + time_dif and action_time < self.enemy_time_to_ball - time_dif):
                            self.clear()
                            break

    def can_any_foe_aerial(self):
        len_fs = str(len(self.foes) - 1)
        for car in self.foes:
            if car.profile[len_fs][3] > self.profiler_threshold:
                return True
        return False

    def get_shot(self, target=None, weight=None, cap=None, towards=False):
        if self.me.minimum_time_to_ball == 7:
            return

        if weight is None:
            weight = self.get_weight(target, towards=towards)

        can_aerial = self.aerials
        can_double_jump = self.double_jump
        can_jump = self.jump
        can_ground = self.ground_shot

        if self.game_mode != "heatseeker":
            if self.num_friends == 0 and self.num_foes == 1:
                can_aerial = can_aerial and (self.is_own_goal or (self.me.location.z > 300 and self.me.airborne) or target is self.best_shot or not self.can_any_foe_aerial())

            if self.num_friends == 0:
                self_intercept_location = self.ball_prediction_struct.slices[self.min_intercept_slice].physics.location
                self_intercept_location = Vector(abs(self_intercept_location.x), self_intercept_location.y * self.friend_team_side)
                can_double_jump = can_double_jump and (self_intercept_location.x < 1300 or self_intercept_location.y > 3840)

        if not can_aerial and not can_double_jump and not can_jump and not can_ground:
            return

        if target is self.anti_shot and self.me.location.y * self.friend_team_side > 5120:
            target = None
            
        if target is not None:
            shot_finder = find_shot if towards else find_any_shot_towards
            shot = find_shot(self, target, weight=weight, cap_=6 if cap is None else cap, can_aerial=can_aerial, can_double_jump=can_double_jump, can_jump=can_jump, can_ground=can_ground)
        else:
            shot = find_any_shot(self, cap_=4 if cap is None else cap, can_aerial=can_aerial, can_double_jump=can_double_jump, can_jump=can_jump, can_ground=can_ground)

        if shot is not None:
            return shot

    def get_stack_name(self):
        if self.is_clear():
            return None

        return self.stack[0].__class__.__name__

    def shoot_from(self, shot):
        if self.shooting:
            self.upgrade_shot(shot)
        else:
            self.clear()
            self.send_quick_chat(QuickChats.CHAT_TEAM_ONLY, QuickChats.Information_IGotIt)
            self.push(shot)

    def smart_shot(self, target=None, weight=None, cap=None, towards=False):
        shot = self.get_shot(target, weight, cap, towards)
        if shot is not None:
            self.shoot_from(shot)
            return True
        return False

    def upgrade_shot(self, shot):
        current_shot_name = self.get_stack_name()
        new_shot_name = shot.__class__.__name__

        if new_shot_name is current_shot_name:
            self.stack[0].update(shot)
        else:
            self.clear()
            self.shooting = True
            self.push(shot)

    def kickoff_check(self, pair):
        return almost_equals(pair[0], self.me.location.x, 50) and almost_equals(pair[1], abs(self.me.location.y), 50)

    def defensive_kickoff(self):
        if self.game_mode == "heatseeker":
            self.kickoff_done = True
            return

        if self.kickoff_check(self.kickoff_back) or self.boost_amount != "default":
            self.backcheck()
            self.kickoff_done = True
        elif self.kickoff_check(self.kickoff_left) or self.kickoff_check(self.kickoff_right):
            self.push(corner_kickoff_boost())
        elif self.kickoff_check(self.kickoff_back_left) or self.kickoff_check(self.kickoff_back_right):
            self.push(back_offset_kickoff_boost())
        else:
            self.print("Unknown kickoff position; skipping")
            self.kickoff_done = True
            return

        self.send_quick_chat(QuickChats.CHAT_TEAM_ONLY, QuickChats.Information_Defending)
        self.print("Defending!")

        send_comm(self, {
            "match_defender": True
        })

    def offensive_kickoff(self):
        if self.game_mode == "dropshot" or self.game_mode == "hoops":
            self.print(f"Skipping the kickoff due to the gamemode")
            self.kickoff_done = True
            return

        if self.game_mode == "heatseeker":
            self.push(generic_kickoff())
        elif self.kickoff_check(self.kickoff_back):
            if not almost_equals(-self.gravity.z, 650, 50) or self.boost_amount != "default" or self.cheating:
                self.push(generic_kickoff())
            else:
                self.push(back_kickoff())
        elif self.kickoff_check(self.kickoff_left):
            if not almost_equals(-self.gravity.z, 650, 50) or self.boost_amount != "default" or self.cheating:
                self.push(generic_kickoff())
            else:
                self.push(corner_kickoff(-1))
        elif self.kickoff_check(self.kickoff_right):
            if not almost_equals(-self.gravity.z, 650, 50) or self.boost_amount != "default" or self.cheating:
                self.push(generic_kickoff())
            else:
                self.push(corner_kickoff(1))
        elif self.kickoff_check(self.kickoff_back_left) or self.kickoff_check(self.kickoff_back_right):
            if not almost_equals(-self.gravity.z, 650, 50) or self.boost_amount != "default" or self.cheating:
                self.push(generic_kickoff())
            else:
                self.push(back_offset_kickoff())
        else:
            self.print("Unknown kickoff position; skipping")
            self.kickoff_done = True
            return

        self.send_quick_chat(QuickChats.CHAT_TEAM_ONLY, QuickChats.Information_IGotIt)

        self.print("I got it!")

        send_comm(self, {
            "attacking": True
        })

    def backcheck(self, clear_on_valid=False):
        if self.is_clear() or clear_on_valid:
            routine_shadow = shadow()
            if routine_shadow.is_viable(self):
                if clear_on_valid: self.clear()

                self.push(routine_shadow)
                return True

            routine_retreat = retreat()
            if routine_retreat.is_viable(self):
                if clear_on_valid: self.clear()

                self.push(routine_retreat)
                return True

        return False

    def boost_viability_check(self):
        if self.is_own_goal:
            self.clear()
            return

        min_foe = self.closest_foes[0] if self.num_foes > 0 else None
        ignore_foe = min_foe is None or min_foe.minimum_time_to_ball == 7
        min_seconds = math.inf

        enemy_post = retreat().get_target(self)
        shadow_routine = shadow()
        friend_post = shadow_routine.get_target(self) if shadow_routine.is_viable(self) else enemy_post

        if not ignore_foe:
            enemy_intercept_location = self.ball_prediction_struct.slices[self.future_ball_location_slice].physics.location
            enemy_intercept_location = Vector(enemy_intercept_location.x, enemy_intercept_location.y, enemy_intercept_location.z)

            if enemy_intercept_location.flat_dist(enemy_post) < 2560:
                return

            dist_to_goal = enemy_intercept_location.dist(enemy_post)
            min_foe_speed = min_foe.location.dist(enemy_intercept_location) / self.enemy_time_to_ball
            min_seconds = dist_to_goal * 1.2 / min_foe_speed
            min_seconds = dist_to_goal / (min_foe_speed * (1-(0.0305/120)) ** (120*min_seconds))

            foe_factor = max(abs(min_foe.local_velocity().angle2D(min_foe.local_location(self.ball.location))), 1)
            min_seconds += min_foe.minimum_time_to_ball * foe_factor

        car_mag = self.me.velocity.magnitude()
        car_mag_adj = (car_mag + 2300) / 2
        # car_speed = self.me.local_velocity().x
        turn_rad = turn_radius(car_mag)
        car_z = self.me.location.z - self.me.hitbox.height / 2

        boost = self.stack[0].boost
        angle = boost.location.angle2D(self.me.forward)
        time = (angle * turn_rad / car_mag) if car_mag > 400 else (1.8 * (angle / math.pi))
        dist = boost.location.flat_dist(self.me.location) + (car_z - 17)
        if dist > 1280:
            time += (dist - 1280) / car_mag_adj
            dist = 1280
        dist += boost.location.flat_dist(friend_post)

        time += dist / ((car_mag + 1410) / 2)

        if time > min_seconds:
            self.clear()

    def goto_nearest_boost(self, only_small=False, clear_on_valid=False):
        self.send_quick_chat(QuickChats.CHAT_TEAM_ONLY, QuickChats.Information_NeedBoost)

        if self.is_own_goal:
            return

        # principle
        # if we can get to the boost and then back to into position before out opponent can get to the ball and then the ball can get to our net, the boost is marked as a viable option
        # out of all the viable options, we pick the option that's closest to our own car
        # however, we must respect the only_small clause as well as prioritize large boosts over the small pads, as well as rule out pads that our tm8s are going for (provided by their TMCP packets)

        min_foe = self.closest_foes[0] if self.num_foes > 0 else None
        ignore_foe = min_foe is None or min_foe.minimum_time_to_ball == 7
        min_seconds = math.inf

        enemy_post = retreat().get_target(self)
        shadow_routine = shadow()
        friend_post = shadow_routine.get_target(self) if shadow_routine.is_viable(self) else enemy_post

        if not ignore_foe:
            enemy_intercept_location = self.ball_prediction_struct.slices[self.future_ball_location_slice].physics.location
            enemy_intercept_location = Vector(enemy_intercept_location.x, enemy_intercept_location.y, enemy_intercept_location.z)

            if enemy_intercept_location.flat_dist(enemy_post) < 2560:
                return

            dist_to_goal = enemy_intercept_location.dist(enemy_post)
            min_foe_speed = min_foe.location.dist(enemy_intercept_location) / self.enemy_time_to_ball
            min_seconds = dist_to_goal * 1.2 / min_foe_speed
            min_seconds = dist_to_goal / (min_foe_speed * (1-(0.0305/120)) ** (120*min_seconds))

            foe_factor = max(abs(min_foe.local_velocity().angle2D(min_foe.local_location(self.ball.location))), 1)
            min_seconds += min_foe.minimum_time_to_ball * foe_factor

        claimed_boosts = tuple(friend.tmcp_action['target'] for friend in self.alive_friends if friend.tmcp_action is not None and friend.tmcp_action['type'] == "BOOST")
        active_unclaimed_boosts = tuple(boost for boost in self.boosts if boost.active and boost.index not in claimed_boosts)
        big_pads = (boost for boost in active_unclaimed_boosts if boost.large)
        small_pads = (boost for boost in active_unclaimed_boosts if not boost.large)
        car_mag = self.me.velocity.magnitude()
        car_mag_adj = (car_mag + 2300) / 2
        # car_speed = self.me.local_velocity().x
        turn_rad = turn_radius(car_mag)
        car_z = self.me.location.z - self.me.hitbox.height / 2

        for boosts in ((big_pads if not only_small else ()), small_pads):
            viable_boosts = []
            for boost in boosts:
                angle = boost.location.angle2D(self.me.forward)
                time = (angle * turn_rad / car_mag) if car_mag > 400 else (1.8 * (angle / math.pi))
                dist = boost.location.flat_dist(self.me.location) + (car_z - 17)
                if dist > 1280:
                    time += (dist - 1280) / car_mag_adj
                    dist = 1280
                dist += boost.location.flat_dist(friend_post)

                time += dist / ((car_mag + 1410) / 2)

                if time < min_seconds:
                    viable_boosts.append({"pad": boost, "time": time})

            if len(viable_boosts) > 0:
                if clear_on_valid: self.clear()
                self.push(goto_boost(min(viable_boosts, key=lambda boost: boost["time"])["pad"]))


if __name__ == "__main__":
    run_bot(VirxEB)
