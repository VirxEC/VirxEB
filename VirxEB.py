import itertools

# from rlbot.utils.game_state_util import CarState, BallState, GameState, Physics, Vector3
from rlbot.utils.structures.quick_chats import QuickChats

import virxrlcu
from util.agent import Vector, VirxERLU, math
from util.replays import back_kickoff
from util.routines import (ball_recovery, block_ground_shot, boost_down,
                           corner_kickoff, face_target, generic_kickoff, goto,
                           goto_boost, recovery, retreat, shadow, short_shot,
                           wave_dash)
from util.tools import find_any_shot, find_shot
from util.utils import (almost_equals, cap, get_weight, peek_generator,
                        send_comm, side, sign)


class VirxEB(VirxERLU):
    def init(self):
        foe_team = -1 if self.team == 1 else 1
        team = -foe_team

        self.defensive_shots = (
            (self.foe_goal.left_post, self.foe_goal.right_post),
            (Vector(4096, foe_team * 3968, 1900), Vector(2944, foe_team * 5120, 1900)),
            (Vector(-4096, foe_team * 3968, 1900), Vector(-2944, foe_team * 5120, 1900))
        )

        self.panic_shots = (
            (Vector(3100 * team, team * 3620, 1000), Vector(3100 * team, team * 5120, 1000)),
            (Vector(-3100 * team, team * 3620, 1000), Vector(-3100 * team, team * 5120, 1000))
        )

        self.offensive_shots = (
            (self.foe_goal.left_post, self.foe_goal.right_post),
            (Vector(foe_team * 893, foe_team * 5120, 100), Vector(foe_team * 893, foe_team * 4720, 320)),
            (Vector(-foe_team * 893, foe_team * 5120, 100), Vector(-foe_team * 893, foe_team * 4720, 320))
        )

        self.best_shot = (Vector(foe_team * 650, foe_team * 5125, 320), Vector(-foe_team * 650, foe_team * 5125, 320))

        self.max_shot_weight = 4
        self.playstyles_switch = {
            'ground': {
                self.playstyles.Defensive: self.defend_ground,
                self.playstyles.Neutral: self.neutral_ground,
                self.playstyles.Offensive: self.attack_ground
            },
            'air': {
                self.playstyles.Defensive: self.defend_air,
                self.playstyles.Neutral: self.neutral_air,
                self.playstyles.Offensive: self.attack_air
            }
        }

        self.panic_switch = {
            self.playstyles.Defensive: 1920,
            self.playstyles.Offensive: 1280,
            self.playstyles.Neutral: 640
        }

    def test(self):
        # Block ground shot testing
        """
        if self.is_clear():
            if abs(self.ball.location.y) > 1000 and abs(self.me.location.y) > abs(self.ball.location.y):
                bgs = block_ground_shot()
                if bgs.is_viable(self):
                    self.push(bgs)
                    return
            else:
                self.backcheck()
        """
        # Backcheck testing
        """
        self.dbg_2d(self.ball.location)

        if self.is_clear():
            self.backcheck()
        """
        # Shot testing
        """
        if self.shooting:
            # if self.odd_tick == 0:
            #     self.smart_shot(self.best_shot, cap=6)
            pass
        else:
            if self.is_clear() and not self.me.airborne and (self.me.boost < 60 or not self.smart_shot(self.best_shot, cap=6)) and self.me.location.flat_dist(self.debug_vector * side(self.team)) > 250:
                self.push(face_target(ball=True))
                self.push(goto(self.debug_vector.flatten() * side(self.team), brake=True))

            if self.ball.location.z < 250 and not self.predictions['goal']:
                ball_state = BallState(Physics(location=Vector3(0, -4096 * side(self.team), self.ball.location.z), velocity=Vector3(0, 0, 2000), angular_velocity=Vector3(0, 0, 0)))
                self.set_game_state(GameState(ball=ball_state))
        """
        # Any shot testing
        """
        if self.shooting:
            # if self.odd_tick == 0:
            #     self.smart_shot(self.best_shot, cap=6)
            pass
        else:
            if self.is_clear() and not self.me.airborne and (self.me.boost < 60 or not self.smart_shot(cap=6)) and self.me.location.flat_dist(self.debug_vector * side(self.team)) > 250:
                self.push(face_target(ball=True))
                self.push(goto(self.debug_vector.flatten() * side(self.team), brake=True))

            if self.ball.location.z < 250:
                ball_state = BallState(Physics(location=Vector3(0, 0, self.ball.location.z), velocity=Vector3(0, 0, 2000), angular_velocity=Vector3(0, 0, 0)))
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
        if not self.shooting and self.ball.location.z < 100 and not self.predictions['goal']:
            ball_state = BallState(Physics(location=Vector3(self.debug_vector.x * side(self.team), self.debug_vector.y * side(self.team), self.ball.location.z), velocity=Vector3(0, 0, 2000), angular_velocity=Vector3(0, 0, 0)))
            game_state = GameState(ball=ball_state)
            self.set_game_state(game_state)

        if self.is_clear():
            self.push(ceiling_shot())
        """

    def time_to_ball(self, car):
        if car.demolished:
            return self.time + 7

        me = (
            car.location.tuple(),
            car.forward.tuple(),
            car.boost if self.boost_amount != 'unlimited' else 100000,
            car.local_velocity().x
        )

        game_info = (
            self.best_shot_value,
            self.boost_accel
        )

        me_a = (
            me[0],
            car.velocity.tuple(),
            car.up.tuple(),
            me[1],
            car.hitbox.tuple(),
            1 if car.airborne else -1,
            me[2]
        )

        gravity = self.gravity.tuple()

        ground = not car.airborne or car.location.z < 300

        # check every 12th slice
        for ball_slice in self.ball_prediction_struct.slices[12::6]:
            intercept_time = ball_slice.game_seconds
            time_remaining = intercept_time - self.time

            if time_remaining <= 0:
                break

            ball_location = (ball_slice.physics.location.x, ball_slice.physics.location.y, ball_slice.physics.location.z)

            if abs(ball_location[1]) > 5212.75:
                break

            if ground:
                shot = virxrlcu.parse_slice_for_ground_shot(time_remaining, *game_info, ball_location, me_a)

                if shot['found'] == 1:
                    return time_remaining

                shot = virxrlcu.parse_slice_for_jump_shot(time_remaining - 0.3, *game_info, ball_location, *me)

                if shot['found'] == 1:
                    return time_remaining
            
                shot = virxrlcu.parse_slice_for_double_jump(time_remaining - 0.3, *game_info, ball_location, *me)

                if shot['found'] == 1:
                    return time_remaining

            shot = virxrlcu.parse_slice_for_aerial_shot(time_remaining, *game_info, gravity, ball_location, me_a)

            if shot['found'] == 1:
                return time_remaining

        return 7
   
    def run(self):
        # predictions
        len_friends = len(self.friends)
        can_shoot = True

        if len(self.foes) > 0:
            foe_distances = tuple(self.ball.location.flat_dist(foe.location) for foe in self.foes if not foe.demolished)
            self_dist = self.ball.location.flat_dist(self.me.location)
            if len(foe_distances) > 0:
                if self.odd_tick % 2 == 0:
                    self.predictions['enemy_time_to_ball'] = min(tuple(self.time_to_ball(foe) for foe in self.foes))

                self.predictions['closest_enemy'] = min(foe_distances)
            else:
                self.predictions['enemy_time_to_ball'] = 7
                self.predictions['closest_enemy'] = math.inf
        else:
            self.predictions['enemy_time_to_ball'] = 7
            self.predictions['closest_enemy'] = math.inf

        self.dbg_2d(f"Predicted enemy time to ball: {round(self.predictions['enemy_time_to_ball'], 1)}")

        self.predictions['self_from_goal'] = self.friend_goal.location.flat_dist(self.me.location) if not self.me.demolished else math.inf
        self.predictions['self_to_ball'] = self.ball.location.flat_dist(self.me.location) if not self.me.demolished else math.inf

        if not self.predictions['was_down']:
            self.predictions['was_down'] = self.game.friend_score - self.game.foe_score > 1

        if self.odd_tick % 2 == 0:
            if self.goalie:
                self.playstyle = self.playstyles.Defensive
            elif len_friends > 0:
                teammates = tuple(itertools.chain(self.friends, [self.me]))

                self.predictions["team_from_goal"] = sorted(tuple(self.friend_goal.location.flat_dist(teammate.location) if not teammate.demolished else math.inf for teammate in teammates))
                self.predictions["team_to_ball"] = sorted(tuple(self.ball.location.flat_dist(teammate.location) if not teammate.demolished else math.inf for teammate in teammates))

                if len_friends >= 2 and can_shoot:
                    can_shoot = self.predictions['self_from_goal'] != self.predictions["team_from_goal"][0]

                ball_loc_y = self.ball.location.y * side(self.team)

                if ball_loc_y < 2560:
                    # If we're down or up by 2 goals in 2's, then start playing more defensive
                    self_time_to_ball = self.time_to_ball(self.me) * 1.05
                    team_time_to_ball = min(tuple(self.time_to_ball(teammate) for teammate in self.friends)) * 1.05
                    if ball_loc_y < -1280 and self_time_to_ball < team_time_to_ball and self.predictions['self_from_goal'] != self.predictions["team_from_goal"][0]:
                        self.playstyle = self.playstyles.Offensive if len_friends > 1 or (len_friends == 1 and (self.predictions['was_down'] or abs(self.game.friend_score - self.game.foe_score) <= 1)) else self.playstyles.Neutral
                    elif self.predictions['self_from_goal'] == self.predictions["team_from_goal"][0]:
                        self.playstyle = self.playstyles.Defensive if self.gravity.z < -550 and len_friends > 1 or (len_friends == 1 and (self.predictions['was_down'] or abs(self.game.friend_score - self.game.foe_score) > 1)) else self.playstyles.Neutral
                    else:
                        self.playstyle = self.playstyles.Neutral
                else:
                    self.playstyle = self.playstyles.Defensive
            else:
                self_time_to_ball = self.time_to_ball(self.me) * 1.05

                if self.ball.location.y * side(self.team) < 640:
                    self.playstyle = self.playstyles.Offensive if self_time_to_ball < self.predictions['enemy_time_to_ball'] else self.playstyles.Neutral
                else:
                    self.playstyle = self.playstyles.Neutral if self_time_to_ball < self.predictions['enemy_time_to_ball'] else self.playstyles.Defensive

            is_own_goal = False
            is_goal = False

            if self.ball_prediction_struct is not None:
                for ball_slice in self.ball_prediction_struct.slices[30::12]:
                    location = ball_slice.physics.location.y * side(self.team)

                    if location >= 5212.75:
                        is_own_goal = True
                        break

                    if location <= -5212.75:
                        is_goal = True
                        break

            if is_own_goal and not self.predictions['own_goal']:
                self.send_quick_chat(QuickChats.CHAT_EVERYONE, QuickChats.Compliments_NiceShot)
            
            if is_goal and not self.predictions['goal']:
                self.send_quick_chat(QuickChats.CHAT_EVERYONE, QuickChats.Reactions_Wow)

            self.predictions["own_goal"] = is_own_goal
            self.predictions["goal"] = is_goal

        if not can_shoot and self.can_shoot is None:
            self.can_shoot = self.time - 2.9

        if self.can_shoot is not None and (self.time - self.can_shoot >= 3 or self.predictions['own_goal']):
            self.can_shoot = None

        # act on the predictions
        
        if not self.kickoff_done:
            if self.is_clear():
                if len(self.friends) > 0:
                    if almost_equals(min(self.predictions['team_to_ball']), self.predictions['self_to_ball'], 5):
                        self.offensive_kickoff()
                    elif almost_equals(max(self.predictions['team_to_ball']), self.predictions['self_to_ball'], 5):
                        self.defensive_kickoff()
                elif almost_equals(self.predictions['closest_enemy'], self.predictions['self_to_ball'], 10):
                    self.offensive_kickoff()
                else:
                    self.defensive_kickoff()
        else:
            if side(self.team) * self.ball.location.y >= self.panic_switch[self.playstyle] or self.predictions['own_goal']:
                for shots in (self.defensive_shots, self.panic_shots):
                    for shot in shots:
                        self.line(*shot, self.renderer.team_color(alt_color=True))

                ball_loc = self.ball.location * side(self.team)
                self_loc = self.me.location * side(self.team)

                ball_f = self.ball_prediction_struct.slices[self.future_ball_location_slice].physics.location
                ball_f = Vector(ball_f.x, ball_f.y, ball_f.z)

                if not self.predictions['own_goal'] and self_loc.y <= ball_loc.y - 50 and not self.shooting and (self.is_clear() or self.stack[0].__class__.__name__ == 'goto_boost') and abs(ball_loc.x) > 1024 and self.backcheck(clear_on_valid=True):
                    return

                # This is a list of all tm8s that are onside
                team_to_ball = [car.location.flat_dist(self.ball.location) for car in self.friends if car.location.y * side(self.team) >= ball_loc.y + 100 and abs(car.location.x) < abs(ball_loc.x) - 250]
                self_to_ball = self.me.location.flat_dist(self.ball.location)
                team_to_ball.append(self_to_ball)
                team_to_ball.sort()

                if not self.shooting or self.shot_weight == -1:
                    if len(team_to_ball) == 1 or team_to_ball[math.ceil(len(team_to_ball) / 2)] + 10 > self_to_ball:
                        self.can_shoot = None

                    fake_own_goal = self.last_ball_location.dist(self.friend_goal.location) > self.ball.location.dist(self.friend_goal.location) and self_loc.y < ball_loc.y and abs(self_loc.x) < abs(ball_loc.x)

                    max_panic_x_ball_loc = 900 if len(self.friends) >= 2 else 1200

                    if self_loc.y > ball_loc.y + 50 and ((ball_loc.x > 0 and ball_loc.x < max_panic_x_ball_loc and self.smart_shot(self.panic_shots[0], cap=3)) or (ball_loc.x < 0 and ball_loc.x > -max_panic_x_ball_loc and self.smart_shot(self.panic_shots[1], cap=3))):
                        return

                    if fake_own_goal or self.predictions['own_goal'] or (len(team_to_ball) > 1 and team_to_ball[math.ceil(len(team_to_ball) / 2)] + 10 > self_to_ball) or (len(team_to_ball) == 1 and self_to_ball < 2560) or (abs(ball_loc.x) < 900 and ball_loc.y > 1280):
                        if ball_loc.y < 1280:
                            for shot in self.defensive_shots:
                                if self.smart_shot(shot):
                                    return

                        if self_loc.y > ball_loc.y and team_to_ball[0] is self_to_ball and self.smart_shot(weight=self.max_shot_weight - 1, cap=6):
                            return

                        if len(self.friends) > 1 and self_loc.y + 100 > ball_loc.y and ((abs(self_loc.x) < abs(ball_loc.x) and min(self.predictions['team_from_goal']) < self.predictions['self_from_goal']) or self.predictions['own_goal']) and self.is_clear():
                            self.push(short_shot(self.foe_goal.location))
                            return

                    self.backcheck()

                elif self.shooting and self.odd_tick == 0:
                    if ball_loc.y < 1280:
                        for i, d_shot in enumerate(self.defensive_shots):
                            shot_weight = get_weight(self, index=i)

                            if shot_weight < self.shot_weight:
                                break

                            shot = self.get_shot(d_shot, weight=shot_weight)

                            if shot is not None:
                                if shot_weight is self.shot_weight:
                                    if shot['intercept_time'] < self.shot_time - 0.05:
                                        self.shoot_from(shot, clear_on_valid=True)
                                        return
                                elif shot['intercept_time'] <= min(self.shot_time + (shot_weight - self.shot_weight / 3), 5):
                                    self.shoot_from(shot, clear_on_valid=True)
                                    return
                    else:
                        shot = None
                        if self.shot_weight is self.max_shot_weight:
                            if self_loc.y > ball_loc.y - 50:
                                if ball_loc.x > 100 and ball_loc.x < 900 and self_loc.x > ball_loc.x:
                                    shot = self.get_shot(self.panic_shots[0], weight=self.max_shot_weight)
                                elif ball_loc.x < -100 and ball_loc.x > -900 and self_loc.x < ball_loc.x:
                                    shot = self.get_shot(self.panic_shots[1], weight=self.max_shot_weight)
                        elif self_loc.y > ball_loc.y and team_to_ball[0] is self_to_ball:
                            shot = self.get_shot(weight=self.max_shot_weight - 1)

                        if shot is not None:
                            if self.shot_weight is shot['weight'] and shot['intercept_time'] < self.shot_time - 0.05:
                                self.shoot_from(shot, clear_on_valid=True)
                                return

                    if self_loc.y <= ball_loc.y - 50 and not self.shooting and (self.is_clear() or self.stack[0].__class__.__name__ == 'goto_boost') and self.backcheck(clear_on_valid=True):
                        return

                ball_f.y = cap(ball_f.y, -5100, 5100)
                if self.is_clear() and ball_f.y * side(self.team) < 3840 and abs(Vector(x=1).angle(self.me.local_location(ball_f))) >= 1:
                    self.push(face_target(target=ball_f))

            elif self.me.airborne:
                self.playstyles_switch['air'][self.playstyle]()

                if self.is_clear():
                    self.push(recovery())
            else:
                self.playstyles_switch['ground'][self.playstyle]()

            self.last_ball_location = self.ball.location
        ""

    def handle_match_comm(self, msg):
        if not self.kickoff_done and msg.get("VirxERLU") is not None and msg['VirxERLU']['team'] is self.team:
            msg = msg['VirxERLU']
            if ((msg.get("match_defender", False) and ((not self.is_clear() and self.stack[0].__class__.__name__ not in {"corner_kickoff", "generic_kickoff"}) or self.is_clear())) or (msg.get("attacking", False) and ((not self.is_clear() and self.stack[0].__class__.__name__ not in {"corner_kickoff"}) or self.is_clear()))) and msg['index'] < self.index:
                self.clear()
                self.goto_nearest_boost()
                self.can_shoot = self.time
                self.kickoff_done = True

    def handle_quick_chat(self, index, team, quick_chat):
        if self.kickoff_done and team is self.team and index is not self.index:
            if quick_chat is QuickChats.Information_IGotIt:
                if side(self.team) * self.ball.location.y < 4200 and not self.predictions['own_goal'] and not self.shooting:
                    self.can_shoot = self.time
                    if side(self.team) * self.ball.location.y < 2560:
                        self.can_shoot += 2.5
                    elif side(self.team) * self.ball.location.y < 750:
                        self.can_shoot += 2
                    else:
                        self.can_shoot += 1

                    if self.shooting and self.shot_weight == -1:
                        self.clear()
                        self.backcheck()
            elif quick_chat is QuickChats.Information_GoForIt:
                if self.playstyle is self.playstyles.Neutral:
                    if self.is_clear() and not self.shooting and self.me.boost >= 36:
                        if not self.smart_shot(self.best_shot, cap=4) and not self.smart_shot(self.offensive_shots[0], cap=3) and not self.smart_shot(cap=6):
                            self.push(short_shot(self.foe_goal.location))

    def defend_ground(self):
        if self.shooting and not self.predictions['own_goal'] and self.ball.location.z * side(self.team) < 750:
            self.clear()

        if self.is_clear():
            ball = self.ball_prediction_struct.slices[self.future_ball_location_slice].physics.location
            ball = Vector(ball.x, ball.y, ball.z)
            if self.predictions['self_from_goal'] > 2560:
                self.backcheck()
            if self.me.boost < 72 and ball.y * side(self.team) < -1280:
                self.goto_nearest_boost(only_small=ball.y * side(self.team) > -2560)
            elif abs(Vector(x=1).angle(self.me.local_location(ball))) >= 1:
                self.push(face_target(target=ball))
            elif self.predictions['self_from_goal'] > 750:
                self.backcheck()

    def defend_air(self):
        if self.odd_tick % 2 == 0 and (self.can_shoot is None or self.predictions['own_goal']):
            shot = self.get_shot(weight=self.max_shot_weight - 1)

            if shot is not None:
                if self.shooting:
                    self.shoot_from(shot)
                else:
                    self.upgrade_shot(shot)

        if (self.is_clear() or self.stack[0].__class__.__name__ == "ball_recovery") and self.boost_amount == 'unlimited' and self.gravity.z > -700 and self.me.location.z > 750 and self.predictions['self_to_ball'] > 2560:
            if not self.is_clear():
                self.clear()

            self.push(boost_down())

    def neutral_ground(self):
        if (self.is_clear() or not self.shooting) and self.odd_tick % 2 == 0:
            if not self.smart_shot(self.best_shot, cap=5) and (not self.smart_shot(self.offensive_shots[0], cap=3) or len(self.friends) == 0 or self.can_shoot is not None) and self.is_clear():
                if self.ball.location.y * side(self.team) > (-1280 if len(self.friends) == 0 else 2580):
                    for i, shot in enumerate(self.defensive_shots[1:]):
                        shot = self.get_shot(shot, cap=2)

                        if shot is not None:
                            if self.shooting:
                                self.upgrade_shot(shot)
                            else:
                                self.shoot_from(shot)
                            return

                    if self.me.location.y * side(self.team) > self.ball.location.y * side(self.team) and self.smart_shot(cap=2):
                        return

        if self.is_clear():
            if self.me.boost < 36:
                self.goto_nearest_boost()
                return

            if self.is_clear():
                self.backcheck()
        elif self.odd_tick % 2 == 0 and self.shooting:
            shot = self.get_shot(self.best_shot)
            if shot is None and self.shot_weight is self.max_shot_weight and len(self.friends) > 0:
                shot = self.get_shot(self.offensive_shots[0])

            if shot is not None:
                self.upgrade_shot(shot)
                return

            if self.ball.location.y * side(self.team) > -2560:
                for i, shot in enumerate(self.defensive_shots[1:]):
                    shot_weight = get_weight(self, index=i)

                    if shot_weight < self.shot_weight:
                        break

                    shot = self.get_shot(shot)

                    if shot is not None:
                        self.upgrade_shot(shot)
                        return

                if self.me.location.y * side(self.team) > self.ball.location.y * side(self.team) and self.smart_shot(cap=2):
                    return

    def neutral_air(self):
        if self.odd_tick % 2 == 0 and (self.can_shoot is None or self.predictions['own_goal']):
            self.line(*self.offensive_shots[0], self.renderer.team_color(alt_color=True))
            shot = self.get_shot(self.best_shot, cap=5)

            if shot is None:
                shot = self.get_shot(self.offensive_shots[0], cap=3)

                if shot is None:
                    self.line(*self.offensive_shots[1], self.renderer.team_color(alt_color=True))
                    shot = self.get_shot(self.offensive_shots[1], weight=self.max_shot_weight - 1, cap=3)

                    if shot is None:
                        self.line(*self.offensive_shots[2], self.renderer.team_color(alt_color=True))
                        shot = self.get_shot(self.offensive_shots[2], weight=self.max_shot_weight - 1, cap=3)

            if shot is not None:
                if self.shooting:
                    self.upgrade_shot(shot)
                else:
                    self.shoot_from(shot)
                return

        if (self.is_clear() or self.stack[0].__class__.__name__ == "ball_recovery") and self.boost_amount == 'unlimited' and self.gravity.z > -700 and self.me.location.z > 750 and self.predictions['self_to_ball'] > 2560:
            if not self.is_clear():
                self.clear()

            self.push(boost_down())

    def attack_ground(self):
        if not self.shooting or self.shot_weight == -1:
            if self.is_clear():
                if self.predictions['goal'] or (self.foe_goal.location.dist(self.ball.location) <= 5120 and (self.predictions['closest_enemy'] > 5120 or self.foe_goal.location.dist(self.me.location) < self.predictions['closest_enemy'] + 250)) or self.foe_goal.location.dist(self.ball.location) < 750:
                    self.line(*self.best_shot, self.renderer.team_color(alt_color=True))
                    shot = self.get_shot(self.best_shot, cap=4)

                    if shot is not None:
                        self.shoot_from(shot, clear_on_valid=True)
                elif self.can_shoot is None or self.predictions['own_goal']:
                    shots = [self.offensive_shots[0]]
                    if self.ball.location.x * side(not self.team) > 1000:
                        shots.append(self.offensive_shots[1])
                    elif self.ball.location.x * side(not self.team) < -1000:
                        shots.append(self.offensive_shots[2])

                    for o_shot in shots:
                        self.line(*o_shot, self.renderer.team_color(alt_color=True))

                    for i, o_shot in enumerate(shots):
                        shot = self.get_shot(self.best_shot, cap=4) if i == 0 else None

                        if shot is None:
                            shot = self.get_shot(o_shot, cap=3)

                        if shot is not None:
                            self.shoot_from(shot, defend=False, clear_on_valid=True)
                            return

                    if self.ball.location.y * side(self.team) > -1280:
                        for i, shot in enumerate(self.defensive_shots[1:]):
                            shot = self.get_shot(shot, cap=3)

                            if shot is not None:
                                self.shoot_from(shot)
                                return

                        if self.me.location.y * side(self.team) > self.ball.location.y * side(self.team) and self.smart_shot(cap=3):
                            return

            if self.is_clear():
                if self.me.boost < 24:
                    self.goto_nearest_boost()

                self.send_quick_chat(QuickChats.CHAT_EVERYONE, QuickChats.Information_GoForIt)
                self.backcheck()
        elif self.odd_tick % 2 == 0 and self.shooting and (self.can_shoot is None or self.predictions['own_goal']):
            if self.predictions['goal'] or (self.foe_goal.location.dist(self.ball.location) <= 1500 and (self.predictions['closest_enemy'] > 1400 or self.foe_goal.location.dist(self.me.location) < self.predictions['closest_enemy'] + 250)):
                self.line(*self.best_shot, self.renderer.team_color(alt_color=True))
                shot = self.get_shot(self.best_shot)

                if shot is not None:
                    if self.shooting:
                        self.upgrade_shot(shot)
                    else:
                        self.shoot_from(shot)
            elif self.odd_tick == 0:
                if self.ball.location.y * side(self.team) > -1280:
                    for i, shot in enumerate(self.defensive_shots[1:]):
                        shot_weight = get_weight(self, index=i)

                        if shot_weight < self.shot_weight:
                            break

                        shot = self.get_shot(shot)

                        if shot is not None:
                            if self.shooting:
                                self.upgrade_shot(shot)
                            else:
                                self.shoot_from(shot)
                            return

                    if self.me.location.y * side(self.team) > self.ball.location.y * side(self.team) and self.smart_shot():
                        return
                else:
                    shots = [self.offensive_shots[0]]
                    if self.ball.location.x * side(not self.team) > 1000:
                        shots.append(self.offensive_shots[1])
                    elif self.ball.location.x * side(not self.team) < -1000:
                        shots.append(self.offensive_shots[2])

                    for o_shot in shots:
                        self.line(*o_shot, self.renderer.team_color(alt_color=True))

                    for i, o_shot in enumerate(shots):
                        shot = None
                        if i == 0:
                            shot_weight = self.max_shot_weight + 1
                            shot = self.get_shot(self.best_shot, weight=shot_weight)

                        if shot is None:
                            shot_weight = get_weight(self, index=i)

                            if shot_weight < self.shot_weight:
                                break

                            shot = self.get_shot(o_shot, weight=shot_weight)

                        if shot is not None:
                            if self.shooting:
                                self.upgrade_shot(shot)
                            else:
                                self.shoot_from(shot)
                            return

        if (self.is_clear() or not self.shooting) and self.odd_tick == 0:
            if not self.smart_shot(self.best_shot, cap=4 if self.can_shoot is None else 1) and (self.can_shoot is not None or not self.smart_shot(self.offensive_shots[0], cap=3)) and self.is_clear() and self.ball.location.y * side(self.team) > self.me.location.y - 250:
                self.send_quick_chat(QuickChats.CHAT_EVERYONE, QuickChats.Information_GoForIt)
                self.backcheck()

    def attack_air(self):
        if self.odd_tick % 2 == 0 and (self.can_shoot is None or self.predictions['own_goal']):
            if self.predictions['goal'] or (self.foe_goal.location.dist(self.ball.location) <= 1500 and (self.predictions['closest_enemy'] > 1400 or self.foe_goal.location.dist(self.me.location) < self.predictions['closest_enemy'] + 250)):
                self.line(*self.best_shot, self.renderer.team_color(alt_color=True))
                shot = self.get_shot(self.best_shot, cap=2)

                if shot is not None:
                    if self.shooting:
                        self.upgrade_shot(shot)
                    else:
                        self.shoot_from(shot)
                    return
            else:
                self.line(*self.offensive_shots[0], self.renderer.team_color(alt_color=True))

                shot = self.get_shot(self.best_shot, weight=self.max_shot_weight + 1, cap=2)

                if shot is None:
                    shot = self.get_shot(self.offensive_shots[0], weight=self.max_shot_weight, cap=2)

                if shot is not None:
                    if self.shooting:
                        self.upgrade_shot(shot)
                    else:
                        self.shoot_from(shot)
                    return

        if (self.is_clear() or self.stack[0].__class__.__name__ == "ball_recovery") and self.boost_amount == 'unlimited' and self.gravity.z > -700 and self.me.location.z > 750 and self.predictions['self_to_ball'] > 2560:
            if not self.is_clear():
                self.clear()

            self.push(boost_down())
        elif self.is_clear():
            self.send_quick_chat(QuickChats.CHAT_EVERYONE, QuickChats.Information_GoForIt)
            self.backcheck()

    def get_shot(self, target=None, weight=None, cap=None):
        if self.can_shoot is None or self.predictions['own_goal'] or (self.playstyle is self.playstyles.Neutral and target is self.best_shot):
            if weight is None:
                weight = get_weight(self, target)

            can_aerial = self.aerials
            can_double_jump = self.double_jump and not self.me.airborne
            can_jump = self.jump and not self.me.airborne
            can_ground = self.ground_shot and not self.me.airborne

            shot = find_shot(self, target, weight=weight, cap_=6 if cap is None else cap, can_aerial=can_aerial, can_double_jump=can_double_jump, can_jump=can_jump, can_ground=can_ground) if target is not None else find_any_shot(self, cap_=3 if cap is None else cap, can_aerial=can_aerial, can_double_jump=can_double_jump, can_jump=can_jump, can_ground=can_ground)

            if shot is not None:
                return {
                    "weight": weight,
                    "intercept_time": shot.intercept_time,
                    "is_best_shot": target is self.best_shot,
                    "shot": shot
                }

    def shoot_from(self, shot, defend=True, clear_on_valid=False):
        if self.can_shoot is None or self.predictions['own_goal'] or (self.playstyle is self.playstyles.Neutral and shot['is_best_shot']):
            if (defend and not self.shooting and not self.is_clear()) or clear_on_valid:
                self.clear()

            if self.is_clear():
                self.shooting = True
                self.shot_time = shot['intercept_time']
                self.shot_weight = shot['weight']

                self.push(shot['shot'])
                self.send_quick_chat(QuickChats.CHAT_TEAM_ONLY, QuickChats.Information_IGotIt)

    def smart_shot(self, target=None, weight=None, cap=None):
        shot = self.get_shot(target, weight, cap)
        if shot is not None:
            if self.shooting:
                self.upgrade_shot(shot)
            else:
                self.shoot_from(shot)
            return True
        return False

    def upgrade_shot(self, shot):
        current_shot_name = self.stack[0].__class__.__name__
        new_shot_name = shot.__class__.__name__

        if new_shot_name is current_shot_name:
            self.shot_time = shot['intercept_time']
            self.shot_weight = shot['weight']
            self.stack[0].update(shot)
        else:
            self.clear()

            self.shooting = True
            self.shot_time = shot['intercept_time']
            self.shot_weight = shot['weight']

            self.push(shot['shot'])

    def defensive_kickoff(self):
        self.can_shoot = self.time

        self.print("Defending!")

        send_comm(self, {
            "match_defender": True
        })
        self.send_quick_chat(QuickChats.CHAT_TEAM_ONLY, QuickChats.Information_Defending)
        self.push(retreat())
        self.can_shoot = self.time
        self.kickoff_done = True

    def offensive_kickoff(self):
        # note that the second value may be positive or negative
        left = (-2048 * side(self.team), 2560)
        right = (2048 * side(self.team), 2560)
        back = (0, 4608)
        back_left = (-256 * side(self.team), 3840)
        back_right = (256 * side(self.team), 3840)

        def kickoff_check(pair):
            return almost_equals(pair[0], self.me.location.x, 50) and almost_equals(pair[1], abs(self.me.location.y), 50)

        if kickoff_check(back):
            if not almost_equals(-self.gravity.z, 650, 50) or self.boost_amount != "default":
                self.push(generic_kickoff())
            else:
                self.push(back_kickoff())
        elif kickoff_check(left):
            if not almost_equals(-self.gravity.z, 650, 50) or self.boost_amount != "default":
                self.push(generic_kickoff())
            else:
                self.push(corner_kickoff(-1))
        elif kickoff_check(right):
            if not almost_equals(-self.gravity.z, 650, 50) or self.boost_amount != "default":
                self.push(generic_kickoff())
            else:
                self.push(corner_kickoff(1))
        elif kickoff_check(back_left) or kickoff_check(back_right):
            self.push(generic_kickoff())
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
            ball_slice = self.ball_prediction_struct.slices[self.future_ball_location_slice].physics.location
            ball = Vector(ball_slice.x, ball_slice.y, ball_slice.z)

            routine_shadow = shadow()
            if ((self.playstyle is not self.playstyles.Defensive and ball.y * side(self.team) < 2560) or (len(self.friends) == 0 and ball.dist(self.friend_goal.location) > 2560)) and not self.predictions['own_goal'] and self.me.location.y * side(self.team) > ball.y * side(self.team):
                if self.me.location.flat_dist(routine_shadow.get_target(self)) > 320:
                    if clear_on_valid:
                        self.clear()

                    self.push(routine_shadow)
                    return True
            else:
                routine_retreat = retreat()
                if self.me.location.flat_dist(routine_retreat.get_target(self)) > 320:
                    if clear_on_valid:
                        self.clear()

                    self.push(routine_retreat)
                    return True

        return False

    def goto_nearest_boost(self, only_small=False):
        if self.is_clear():
            self.send_quick_chat(QuickChats.CHAT_TEAM_ONLY, QuickChats.Information_NeedBoost)

            if not only_small:
                if len(self.friends) > 0:
                    large_boosts = (boost for boost in self.boosts if boost.large and boost.active and ((self.playstyle is self.playstyles.Offensive and boost.location.y * side(self.team) < -3000) or (self.playstyle is self.playstyles.Neutral and boost.location.y * side(self.team) > -100) or (self.playstyle is self.playstyles.Defensive and boost.location.y * side(self.team) > 3000)))
                else:
                    ball_slice = self.ball_prediction_struct.slices[self.future_ball_location_slice].physics
                    ball_v = Vector(ball_slice.velocity.x, ball_slice.velocity.y, ball_slice.velocity.z)
                    ball = Vector(ball_slice.location.x, ball_slice.location.y, ball_slice.location.z)
                    ball_y = ball.y * side(self.team)
                    if (ball_v.angle2D(self.foe_goal.location - ball) < 1 and ball_y > 0) or self.predictions['own_goal']:
                        large_boosts = None
                    else:
                        ball_x = sign(ball.x)
                        large_boosts = (boost for boost in self.boosts if boost.large and boost.active and (boost.location.y * side(self.team) > ball_y) and sign(boost.location.x) == ball_x)

                if large_boosts is not None:
                    closest = peek_generator(large_boosts)

                    if closest is not None:
                        closest_distance = closest.location.flat_dist(self.me.location)

                        for item in large_boosts:
                            item_distance = item.location.flat_dist(self.me.location)
                            if item_distance is closest_distance:
                                if item.location.flat_dist(self.me.location) < closest.location.flat_dist(self.me.location):
                                    closest = item
                                    closest_distance = item_distance
                            elif item_distance < closest_distance:
                                closest = item
                                closest_distance = item_distance

                        self.push(goto_boost(closest))
                        return

            small_boosts = (boost for boost in self.boosts if not boost.large and boost.active)

            closest = peek_generator(small_boosts)

            if closest is not None:
                closest_distance = closest.location.flat_dist(self.me.location) + closest.location.flat_dist(self.friend_goal.location) / 2

                for item in small_boosts:
                    item_distance = item.location.flat_dist(self.me.location) + item.location.flat_dist(self.friend_goal.location) / 2

                    if item_distance < closest_distance:
                        item_loc = item.location.y * side(self.team)
                        if (self.playstyle is self.playstyles.Offensive and item_loc < -2560) or (self.playstyle is self.playstyles.Neutral and item_loc < -100) or (self.playstyle is self.playstyles.Defensive and item_loc > 1280):
                            closest = item
                            closest_distance = item_distance

                self.push(goto_boost(closest))
