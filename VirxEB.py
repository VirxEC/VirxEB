import itertools
import math
from enum import Enum

import virx_erlu_rlib as rlru
from rlbot.utils.structures.quick_chats import QuickChats

from cutil.agent import Car
from cutil.eph import PacketHeuristics
from cutil.routines import *
from cutil.tools import find_any_shot, find_shot
from cutil.utils import *
from util.agent import GameTickPacket, Vector, VirxERLU, run_bot
from util.utils import *


class Playstyle(Enum):
    Defensive = -1
    Neutral = 0
    Offensive = 1


class VirxEB(VirxERLU):
    def __init__(self, name, team, index):
        super().__init__(name, team, index)

        self.debug_2d_bool = self.name == self.true_name
        self.debug_vector = Vector(0, 0)
        self.goalie = False
        self.jump = True
        self.double_jump = True
        self.ground_shot = True
        self.aerials = True

    def initialize_agent(self):
        super().initialize_agent()

        self.all: list[Car] = ()
        self.friends: list[Car] = ()
        self.foes: list[Car] = ()
        self.me = Car(self.index)

        if self.game_mode == "heatseeker":
            self.print("Preparing for heatseeker")
            self.goalie = True

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

        self.profiler_last_save = 0
        self.last_ball_touch_time = -1
        self.unpause_timer = -1

        self.enemy_time_to_ball = 7
        self.max_shot_weight = 4
        self.min_intercept_slice = 180
        self.own_goal = {"location": Vector(), "slice": -1}
        self.is_own_goal = False
        self.is_goal = False
        self.side = side(self.team)

        self.packet_heuristics = PacketHeuristics()

    def init(self):
        foe_team = -1 if self.team == 1 else 1
        team = -foe_team

        # note that the second value may be positive or negative
        self.kickoff_left = (-2048 * self.side, 2560)
        self.kickoff_right = (2048 * self.side, 2560)
        self.kickoff_back = (0, 4608)
        self.kickoff_back_left = (-256 * self.side, 3840)
        self.kickoff_back_right = (256 * self.side, 3840)

        self.offensive_shots = (
            (Vector(foe_team * 893, foe_team * 5120, min(self.ball_radius * 2, 321.3875)), Vector(foe_team * -893, foe_team * 5120, max(642.775 - self.ball_radius, 321.3875))),
            (Vector(foe_team * 893, foe_team * 5120, 321.3875), Vector(foe_team * 893, foe_team * 6000, 321.3875)),
            (Vector(foe_team * -893, foe_team * 5120, 321.3875), Vector(foe_team * -893, foe_team * 6000, 321.3875))
        )

        self.defensive_shots = (
            self.offensive_shots[0],
            (Vector(4096, foe_team * 3968, 1900), Vector(2944, foe_team * 5120, 1900)),
            (Vector(-4096, foe_team * 3968, 1900), Vector(-2944, foe_team * 5120, 1900))
        )

        self.best_shot = (Vector(foe_team * 793, foe_team * 5213, min(self.ball_radius * 3, 321.3875)), Vector(-foe_team * 793, foe_team * 5213, max(642.775 - self.ball_radius * 2, 321.3875)))
        self.anti_shot = (Vector(-team * 1536, team * 5120, 1285.55), Vector(team * 1536, team * 5120, 1285.55))

        if self.name in {"VirxEB", "VirxEMB"}:
            print(f"{self.name}: Check me out at https://www.virxcase.dev!!!")

    def refresh_player_lists(self, packet: GameTickPacket):
        # Useful to keep separate from get_ready because humans can join/leave a match
        self.all = tuple(Car(i, packet) for i in range(packet.num_cars))
        self.update_cars_from_all()

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

        self.packet_heuristics.add_tick(packet)

        if self.name == "VirxEB" and self.time - self.profiler_last_save > 10:
            self.packet_heuristics.save_profiles()
            self.profiler_last_save = self.time

            if len(self.tick_times) != 0:
                avg_mspt = round(sum(self.tick_times) / len(self.tick_times), 3)
                self.print(f"Avg. ms/t: {avg_mspt}")

    def get_weight(self, shot=None, index=None):
        if index is not None:
            return self.max_shot_weight - math.ceil(index / 2)

        if shot is not None:
            if shot is self.best_shot:
                return self.max_shot_weight + 1

            if shot is self.anti_shot:
                return self.max_shot_weight - 2

            for shot_list in (self.offensive_shots, self.defensive_shots):
                try:
                    return self.max_shot_weight - math.ceil(shot_list.index(shot) / 2)
                except ValueError:
                    continue

    def update_predictions(self):
        is_other_tick = self.odd_tick % 2 == 0
        is_forth_tick = self.odd_tick == 0
        if self.num_foes > 0:
            foe_distances = tuple(self.ball.location.flat_dist(foe.location) for foe in self.alive_foes)
            if len(foe_distances) > 0:
                if is_forth_tick or len(self.closest_foes) != self.num_foes:
                    for foe in self.foes:
                        foe.minimum_time_to_ball = self.time_to_ball(foe) if not foe.demolished else 7

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

        self.future_ball_location_slice = min(round(self.enemy_time_to_ball * 120), rlru.get_num_ball_slices() - 1)
        self.dbg_2d(f"Predicted enemy time to ball: {round(self.enemy_time_to_ball, 1)}")

        self.predictions['self_from_goal'] = self.friend_goal.location.flat_dist(self.me.location)
        self.predictions['self_to_ball'] = self.ball.location.flat_dist(self.me.location)

        if not self.predictions['was_down']:
            self.predictions['was_down'] = self.game.friend_score - self.game.foe_score > 1

        if self.num_friends > 0:
            teammates = tuple(itertools.chain(self.alive_friends, [self.me]))

            self.predictions["team_from_goal"] = sorted(tuple(self.friend_goal.location.flat_dist(teammate.location) for teammate in teammates))
            self.predictions["team_to_ball"] = sorted(tuple(self.ball.location.flat_dist(teammate.location) for teammate in teammates))

        if is_forth_tick:
            self.me.minimum_time_to_ball = self.time_to_ball(self.me)
        self.min_intercept_slice = min(round(self.me.minimum_time_to_ball * 120), rlru.get_num_ball_slices() - 1)
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

            for i in range(0, rlru.get_num_ball_slices(), 24):
                ball_slice = rlru.get_slice_index(i)
                location = ball_slice.location[1] * self.side

                if location >= 5212.75:
                    is_own_goal = True
                    break

                if location <= -5212.75:
                    is_goal = True
                    break

            self.own_goal = {"location": Vector(), "slice": -1}

            if is_own_goal:
                for i in range(0, rlru.get_num_ball_slices(), 30):
                    ball_slice = rlru.get_slice_index(i)
                    if ball_slice.location[1] * self.side >= 5200:
                        self.own_goal["location"] = Vector(*ball_slice.location)
                        self.own_goal["slice"] = i
                        break

            if is_own_goal and not self.is_own_goal:
                self.send_quick_chat(QuickChats.CHAT_EVERYONE, QuickChats.Compliments_NiceShot)

            if is_goal and not self.is_goal:
                self.send_quick_chat(QuickChats.CHAT_EVERYONE, QuickChats.Reactions_Wow)

            self.is_own_goal = is_own_goal
            self.is_goal = is_goal

    def test(self):
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
            intercept_location = Vector(*rlru.get_slice_index(self.future_ball_location_slice).location)
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
            if self.is_clear() and not self.me.airborne and (self.me.boost < 60 or not self.smart_shot(self.best_shot, cap=6)) and self.me.location.flat_dist(self.debug_vector * self.side) > 250:
                self.push(face_target(ball=True))
                self.push(goto(self.debug_vector.flatten() * self.side, brake=True))

            if self.ball.location.z < 150:  # and not self.is_goal:
                ball_state = BallState(Physics(location=Vector3(0, -4096 * self.side, self.ball.location.z), velocity=Vector3(0, 0, 2000), angular_velocity=Vector3(0, 0, 0)))
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
    
    def time_to_ball(self, car: Car) -> float:
        if car.demolished:
            return 7

        profile = self.packet_heuristics.predict_car(self.packet_heuristics.get_car(car.name))

        start_slice = max(12, min(round(car.minimum_time_to_ball * 120), rlru.get_num_ball_slices() - 1) - 120) if car.minimum_time_to_ball != 7 else 12
        end_slice = max(12, min(round(5.5 * 120), rlru.get_num_ball_slices() - 1))

        # Construct the target
        options = rlru.TargetOptions(start_slice, end_slice)
        target_id = rlru.new_any_target(car.index, options)
        
        shot = rlru.get_shot_with_target(target_id, True, profile["may_ground_shot"], profile["may_jump_shot"], profile["may_double_jump_shot"], profile["may_aerial"], only=True)

        if shot.found:
            return shot.time - self.time

        return 7

    def run(self):
        # predictions
        self.update_predictions()

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
                time = action['time'] - self.time - 0.5
                if time > 0:
                    any_going_for_ball = True
                    friend_time_to_ball = min(friend_time_to_ball, action['time'] - self.time - 0.5)

        is_closest_to_ball = self.num_friends == 0 or self.me.minimum_time_to_ball < self.friend_times[0].minimum_time_to_ball - 0.5

        if self.goalie:
            self.playstyle = self.playstyles.Defensive
        elif self.num_friends == 0:
            self.playstyle = self.playstyles.Neutral if self.num_foes <= 1 else self.playstyles.Offensive
        elif self.num_friends == 1 or self.is_own_goal:
            if is_closest_to_ball:
                self.playstyle = self.playstyles.Offensive
            else:
                self.playstyle = self.playstyles.Neutral if self.num_foes <= 2 else self.playstyles.Defensive
        elif self.num_friends >= 2:
            if is_closest_to_ball:
                self.playstyle = self.playstyles.Offensive
            else:
                self.playstyle = self.playstyles.Defensive if self.me.minimum_time_to_ball > self.friend_times[-1].minimum_time_to_ball else self.playstyles.Neutral

        self.dbg_3d(self.playstyle.name)
        any_ready = any((f.tmcp_action is not None and f.tmcp_action['type'] in "READY" and (0 < f.tmcp_action['time'] - self.time < 4) for f in self.friends))

        if not self.is_own_goal and self.shooting and self.num_friends > 0:
            self.tmcp_ball_check()

        if not self.is_own_goal and self.num_friends > 0 and (self.playstyle is not self.playstyles.Offensive or any_ready):
            if not self.me.airborne and self.me.boost < 36 and self.boost_amount != "no boost":
                if self.is_clear():
                    self.goto_nearest_boost()

                if not self.is_clear():
                    return

        if not self.is_clear() and self.odd_tick != 0:
            return

        current_shot_weight = self.stack[0].weight if self.shooting and self.stack[0].weight is not None else -1

        if not self.is_own_goal:
            if self.smart_shot(self.best_shot, self.max_shot_weight, 4):
                return

            if is_closest_to_ball:
                for i, shot in enumerate(self.defensive_shots if self.ball.location.y * self.side > -1280 and self.num_friends != 0 else self.offensive_shots):
                    shot_weight = self.get_weight(index=i)

                    if self.shooting and shot_weight < current_shot_weight:
                        break

                    if self.smart_shot(shot, shot_weight, min(friend_time_to_ball, 5)):
                        return

        anti_shot_weight = self.get_weight(self.anti_shot)
        if current_shot_weight <= anti_shot_weight and (self.is_own_goal or is_closest_to_ball or not any_going_for_ball or not any_ready):
            if self.smart_shot(self.anti_shot, anti_shot_weight, friend_time_to_ball):
                return

            if self.is_clear():
                self.push(ShortShot(self.foe_goal.location))

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
            if almost_equals(self.predictions['team_to_ball'][0], self.predictions['self_to_ball'], 5):
                self.offensive_kickoff()
            elif almost_equals(self.predictions['team_to_ball'][-1], self.predictions['self_to_ball'], 5):
                self.defensive_kickoff()
        elif len(self.foes) == 0 or almost_equals(self.predictions['closest_enemy'], self.predictions['self_to_ball'], 10):
            self.offensive_kickoff()
        else:
            self.defensive_kickoff()

    def render_time_predictions(self):
        enemy_intercept_location = Vector(*rlru.get_slice_index(self.future_ball_location_slice).location)
        if self.enemy_time_to_ball != 7:
            self.sphere(enemy_intercept_location, self.ball_radius, self.renderer.red())

        self_intercept_location = Vector(*rlru.get_slice_index(self.min_intercept_slice).location)
        if self.me.minimum_time_to_ball != 7:
            self.sphere(self_intercept_location, self.ball_radius, self.renderer.green())

        cap_location = Vector(*rlru.get_slice_index(round(get_cap(self, 6) * 120) - 1).location)
        self.sphere(cap_location, self.ball_radius, self.renderer.orange())

    def air_recovery(self):
        if (self.is_clear() or self.get_stack_name() == "ball_recovery") and self.gravity.z > -700 and self.me.location.z > 750 and self.predictions['self_to_ball'] > 2560:
            if not self.is_clear():
                self.clear()

            self.push(BoostDown())
            return

        if self.is_clear():
            self.push(routines.Recovery())

    def tmcp_ball_check(self):
        shot_name = self.get_stack_name()
        shot_time = self.stack[0].intercept_time if shot_name not in {'ShortShot', 'short_shot'} else 7

        if shot_time > 1 and (shot_name != "AerialShot" or shot_time > 3):
            is_short_shot = shot_name in {'ShortShot', 'short_shot'}
            for friend in self.friends:
                action = friend.tmcp_action
                if action is not None and action['type'] == "BALL" and action['time'] != -1:
                    if is_short_shot:
                        self.clear()
                        break

                    action_time = action['time'] - self.time
                    if action_time > 0:
                        f_shot_vector = Vector(*action['direction'])
                        time_dif = 0.5 if f_shot_vector.magnitude() != 0 and self.stack[0].shot_vector.angle(f_shot_vector) > 1 else 0.2
                        if (action_time < shot_time - time_dif and shot_time > self.enemy_time_to_ball + time_dif) or (action_time > shot_time + time_dif and action_time < self.enemy_time_to_ball - time_dif):
                            self.clear()
                            break

    def can_any_foe_aerial(self):
        return any(self.packet_heuristics.predict_car(self.packet_heuristics.get_car(car.name))["may_aerial"] for car in self.foes)

    def get_shot(self, target=None, weight=None, cap=None):
        if self.me.minimum_time_to_ball == 7:
            return

        if weight is None:
            weight = self.get_weight(target)

        can_aerial = self.aerials
        can_double_jump = self.double_jump
        can_jump = self.jump
        can_ground = self.ground_shot

        if self.num_friends == 0 and self.num_foes == 1:
            can_aerial = can_aerial and (self.is_own_goal or (self.me.location.z > 300 and self.me.airborne) or target is self.best_shot or not self.can_any_foe_aerial())

        if self.num_friends == 0:
            self_intercept_location = rlru.get_slice_index(self.min_intercept_slice).location
            self_intercept_location = Vector(abs(self_intercept_location[0]), self_intercept_location[1] * self.side)
            can_double_jump = can_double_jump and (self_intercept_location.x < 1300 or self_intercept_location.y > 3840)

        if not can_aerial and not can_double_jump and not can_jump and not can_ground:
            return

        if target is self.anti_shot and self.me.location.y * self.side > 5120:
            target = None
            
        shot = find_shot(self, target, weight=weight, cap_=6 if cap is None else cap, can_aerial=can_aerial, can_double_jump=can_double_jump, can_jump=can_jump, can_ground=can_ground) if target is not None else find_any_shot(self, cap_=4 if cap is None else cap, can_aerial=can_aerial, can_double_jump=can_double_jump, can_jump=can_jump, can_ground=can_ground)

        if shot is not None:
            return shot

    def get_stack_name(self):
        return self.stack[0].__class__.__name__

    def shoot_from(self, shot):
        if self.shooting:
            self.upgrade_shot(shot)
        else:
            self.clear()
            self.send_quick_chat(QuickChats.CHAT_TEAM_ONLY, QuickChats.Information_IGotIt)
            self.push(shot)

    def smart_shot(self, target=None, weight=None, cap=None):
        shot = self.get_shot(target, weight, cap)
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
        if self.kickoff_check(self.kickoff_back) or self.boost_amount != "default" or self.game_mode == "heatseeker":
            self.backcheck()
            self.kickoff_done = True
        elif self.kickoff_check(self.kickoff_left) or self.kickoff_check(self.kickoff_right):
            self.push(CornerKickoffBoost())
        elif self.kickoff_check(self.kickoff_back_left) or self.kickoff_check(self.kickoff_back_right):
            self.push(BackOffsetKickoffBoost())
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
        if self.game_mode == "heatseeker" or self.game_mode == "hoops":
            self.print(f"Skipping the kickoff due to the gamemode")
            self.kickoff_done = True
            return

        if self.kickoff_check(self.kickoff_back):
            if not almost_equals(-self.gravity.z, 650, 50) or self.boost_amount != "default":
                self.push(routines.GenericKickoff())
            else:
                self.push(BackKickoff())
        elif self.kickoff_check(self.kickoff_left):
            if not almost_equals(-self.gravity.z, 650, 50) or self.boost_amount != "default":
                self.push(routines.GenericKickoff())
            else:
                self.push(CornerKickoff(-1))
        elif self.kickoff_check(self.kickoff_right):
            if not almost_equals(-self.gravity.z, 650, 50) or self.boost_amount != "default":
                self.push(routines.GenericKickoff())
            else:
                self.push(CornerKickoff(1))
        elif self.kickoff_check(self.kickoff_back_left) or self.kickoff_check(self.kickoff_back_right):
            if not almost_equals(-self.gravity.z, 650, 50) or self.boost_amount != "default":
                self.push(routines.GenericKickoff())
            else:
                self.push(BackOffsetKickoff())
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
            routine_shadow = routines.Shadow()
            if routine_shadow.is_viable(self):
                if clear_on_valid: self.clear()

                self.push(routine_shadow)
                return True

            routine_retreat = routines.Retreat()
            if routine_retreat.is_viable(self):
                if clear_on_valid: self.clear()

                self.push(routine_retreat)
                return True

        return False

    def goto_nearest_boost(self, only_small=False, clear_on_valid=False):
        self.send_quick_chat(QuickChats.CHAT_TEAM_ONLY, QuickChats.Information_NeedBoost)

        # principle
        # if we can get to the boost and then back to into position before out opponent can get to the ball and then the ball can get to our net, the boost is marked as a viable option
        # out of all the viable options, we pick the option that's closest to our own car
        # however, we must respect the only_small clause as well as prioritize large boosts over the small pads, as well as rule out pads that our tm8s are going for (provided by their TMCP packets)

        min_foe = self.closest_foes[0] if self.num_foes > 0 else None
        ignore_foe = min_foe is None or min_foe.minimum_time_to_ball == 7
        min_seconds = math.inf

        enemy_post = routines.Retreat().get_target(self)
        shadow_routine = routines.Shadow()
        friend_post = shadow_routine.get_target(self) if shadow_routine.is_viable(self) else enemy_post

        if self.is_own_goal:
            return

        if not ignore_foe:
            enemy_intercept_location = Vector(*rlru.get_slice_index(self.future_ball_location_slice).location)

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
                self.push(routines.GoToBoost(min(viable_boosts, key=lambda boost: boost["time"])["pad"]))


if __name__ == "__main__":
    run_bot(VirxEB)
