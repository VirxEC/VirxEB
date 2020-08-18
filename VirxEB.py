from traceback import print_exc

# from rlbot.utils.game_state_util import BallState, GameState, Physics
# from rlbot.utils.game_state_util import Vector3
from rlbot.utils.structures.quick_chats import QuickChats

from util.agent import math, VirxERLU, Vector
from util.routines import goto_boost, ball_recovery, short_shot, generic_kickoff, dynamic_backcheck, retreat, block_ground_shot, corner_kickoff, ceiling_shot
from util.replays import back_kickoff
from util.tools import find_jump_shot, find_aerial, find_any_aerial, find_any_jump_shot
from util.utils import side, almost_equals, send_comm, get_weight, peek_generator


class VirxEB(VirxERLU):
    def init(self):
        self.playstyles_switch = {
            'ground': {
                self.playstyles.Defensive: self.defend_ground,
                self.playstyles.Fancy: self.fancy_ground,
                self.playstyles.Neutral: self.neutral_ground,
                self.playstyles.Offensive: self.attack_ground
            },
            'air': {
                self.playstyles.Defensive: self.defend_air,
                self.playstyles.Fancy: self.fancy_air,
                self.playstyles.Neutral: self.neutral_air,
                self.playstyles.Offensive: self.attack_air
            }
        }

        self.panic_switch = {
            self.playstyles.Defensive: 1920,
            self.playstyles.Offensive: 1280,
            self.playstyles.Neutral: 640,
            self.playstyles.Fancy: 0
        }

    def test(self):
        # Go to nearest boost testing
        """
        if self.is_clear():
            self.goto_nearest_boost()
        """
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
        if not self.shooting and self.ball.location.z < 500:
            ball_state = BallState(Physics(location=Vector3(0, -3000 * side(self.team), self.ball.location.z), velocity=Vector3(0, 0, 2000), angular_velocity=Vector3(0, 0, 0)))
            game_state = GameState(ball=ball_state)
            self.set_game_state(game_state)

        if not self.smart_shot() and self.is_clear():
            target = Vector()
            if self.me.location.dist(target) > 250:
                self.push(goto(target, self.foe_goal.location))
            else:
                self.push(goto(Vector(y=2560 * side(not self.team)), self.foe_goal.location, brake=True))
        """
        # Recovery testing
        """
        self.dbg_2d(f"Has jump: {not self.me.jumped}")
        self.dbg_2d(f"Has dodge: {not self.me.doublejumped}")
        if self.is_clear():
            self.push(boost_upwards() if self.me.location.z < 17.1 else ball_recovery())
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
        self.dbg_2d(abs(Vector(x=1).angle(self.me.local(self.ball.location - self.me.location))))
        if self.is_clear():
            self.push(retreat())

    def run(self):
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
            if self.can_shoot is not None and (self.time - self.can_shoot >= 3 or self.predictions['own_goal']):
                self.can_shoot = None

            # :D
            if side(self.team) * self.ball.location.y >= self.panic_switch[self.playstyle] or self.predictions['own_goal']:
                for shots in (self.defensive_shots, self.panic_shots):
                    for shot in shots:
                        self.line(*shot, self.renderer.team_color(alt_color=True))

                ball_loc = self.ball.location * side(self.team)
                self_loc = self.me.location * side(self.team)

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

                    # What is 175?
                    # 175 is the radius of the ball rounded up (93) plus the half the length of the longest car rounded up (breakout; 66) with an extra 10% then rounded up
                    # Basicly it's the 'is an enemy dribbling the ball' detector
                    if self_loc.y > ball_loc.y and self.predictions['closest_enemy'] <= 175 and (self.predictions['own_goal'] or (len(self.friends) > 0 and min(self.predictions['team_from_goal'])) < self.predictions['self_from_goal']):
                        bgs = block_ground_shot()
                        if bgs.is_viable(self):
                            self.clear()
                            self.push(bgs)
                            return

                    max_panic_x_ball_loc = 900 if len(self.friends) >= 2 else 1200

                    if self_loc.y > ball_loc.y + 50 and ((ball_loc.x > 0 and ball_loc.x < max_panic_x_ball_loc and self.smart_shot(self.panic_shots[0], cap=2)) or (ball_loc.x < 0 and ball_loc.x > -max_panic_x_ball_loc and self.smart_shot(self.panic_shots[1], cap=2))):
                        return

                    if self.predictions['own_goal'] or (len(team_to_ball) > 1 and team_to_ball[math.ceil(len(team_to_ball) / 2)] + 10 > self_to_ball) or (len(team_to_ball) == 1 and self_to_ball < 2560) or (abs(ball_loc.x) < 900 and ball_loc.y > 1280):
                        if ball_loc.y < 1280:
                            for shot in self.defensive_shots:
                                if self.smart_shot(shot, cap=2):
                                    return

                        if self_loc.y > ball_loc.y and team_to_ball[0] is self_to_ball and self.smart_shot(weight=self.max_shot_weight - 1, cap=2):
                            return

                        if self.predictions['own_goal'] and abs(self_loc.x) < abs(ball_loc.x) - 100 and self_loc.y < ball_loc.y - 50:
                            bgs = block_ground_shot()
                            if bgs.is_viable(self):
                                self.clear()
                                self.push(bgs)
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
            elif self.me.airborne:
                self.playstyles_switch['air'][self.playstyle]()

                if self.is_clear():
                    self.push(ball_recovery())
            else:
                self.playstyles_switch['ground'][self.playstyle]()
        ""

    def handle_match_comm(self, msg):
        if msg.get("VirxEB") is not None and msg['VirxEB']['team'] is self.team:
            msg = msg['VirxEB']
            if self.playstyle is self.playstyles.Defensive:
                if msg.get("match_defender") and msg['index'] < self.index:
                    self.playstyle = self.playstyles.Neutral
                    self.clear()
                    self.goto_nearest_boost()

                    self.print("You can defend")
            elif self.playstyle is self.playstyles.Offensive:
                if msg.get("attacking"):
                    self.print("Speed pinch kickoff!")

    def handle_quick_chat(self, index, team, quick_chat):
        try:
            if team is self.team and index is not self.index:
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
        except Exception:
            print_exc()

    def smart_shot(self, target=None, weight=None, cap=None):
        shot = self.get_shot(target, weight=weight, cap=cap)
        if shot is not None:
            self.shoot_from(shot, clear_on_valid=True)
            return True
        return False

    def defend_ground(self):
        if self.shooting and not self.predictions['own_goal'] and self.ball.location.z * side(self.team) < 750:
            self.clear()

        if self.is_clear():
            if self.predictions['self_from_goal'] > 2560:
                self.backcheck(simple=True)
            if self.me.boost < 72 and self.ball.location.y * side(self.team) < -1280:
                self.goto_nearest_boost(only_small=self.ball.location.y * side(self.team) > -2560)
            elif self.predictions['self_from_goal'] > 750:
                self.backcheck(simple=True)

    def defend_air(self):
        if not self.shooting:
            shot = self.get_shot(weight=self.max_shot_weight - 1)

            if shot is not None:
                self.shoot_from(shot, clear_on_valid=True)

    def fancy_ground(self):
        if self.is_clear():
            self.push(ceiling_shot())

    def fancy_air(self):
        if self.odd_tick % 2 == 0 and self.can_shoot is None and self.me.boost > 12 and not self.shooting:
            shot = self.get_shot(self.best_shot)
            if shot is None:
                shot = self.get_shot(self.offensive_shots[0], cap=2.5)

            if shot is None:
                shot = self.get_shot(self.offensive_shots[0], cap=2.5)

            if shot is not None:
                if shot['intercept_time'] < self.shot_time - 0.05:
                    self.shoot_from(shot, clear_on_valid=True)

    def neutral_ground(self):
        if self.is_clear():
            if self.predictions['self_to_ball'] > 3840:
                self.backcheck(clear_on_valid=True)
            elif self.me.boost < 40 and self.ball.location.y * side(self.team) < -1280 and not self.predictions['goal'] and self.ball.location.flat_dist(self.foe_goal.location) > 1280:
                self.goto_nearest_boost()

            if self.is_clear():
                self.backcheck()
        elif self.odd_tick % 2 == 0 and self.shooting and not self.me.airborne and self.can_shoot is None:
            shot = self.get_shot(self.best_shot)
            if shot is None and self.shot_weight is self.max_shot_weight and len(self.friends) > 0:
                shot = self.get_shot(self.offensive_shots[0])

            if shot is not None and shot['intercept_time'] < self.shot_time - 0.05:
                self.shoot_from(shot, clear_on_valid=True)
                return

            if self.ball.location.y * side(self.team) > -2560:
                if self.me.location.y * side(self.team) > self.ball.location.y * side(self.team) and self.smart_shot():
                    return

                for i, shot in enumerate(self.defensive_shots[1:]):
                    shot_weight = get_weight(self, index=i)

                    if shot_weight < self.shot_weight:
                        break

                    shot = self.get_shot(shot)

                    if shot is not None:
                        self.shoot_from(shot, clear_on_valid=True)
                        return

        if (self.is_clear() or not self.shooting) and self.odd_tick == 0:
            if not self.smart_shot(self.best_shot, cap=2 if self.can_shoot is None else 1) and (len(self.friends) == 0 or self.can_shoot is not None or not self.smart_shot(self.offensive_shots[0], cap=2.5)) and self.is_clear():
                if self.ball.location.y * side(self.team) > (-1280 if len(self.friends) == 0 else 2580):
                    if self.me.location.y * side(self.team) > self.ball.location.y * side(self.team) and self.smart_shot(cap=1.5):
                        return

                    for i, shot in enumerate(self.defensive_shots[1:]):
                        shot = self.get_shot(shot, cap=1.5)

                        if shot is not None:
                            self.shoot_from(shot)
                            return

                self.backcheck()

    def neutral_air(self):
        if self.odd_tick % 2 == 0 and self.can_shoot is None and self.me.boost > 12 and not self.shooting:
            shot = self.get_shot(self.best_shot)
            if shot is None:
                shot = self.get_shot(self.offensive_shots[0], cap=2.5)

            if shot is not None:
                if shot['intercept_time'] < self.shot_time - 0.05:
                    self.shoot_from(shot, clear_on_valid=True)

    def attack_ground(self):
        if not self.shooting or self.shot_weight == -1:
            if self.me.boost == 0:
                self.backcheck(clear_on_valid=True)
            else:
                if self.predictions['goal'] or (self.foe_goal.location.dist(self.ball.location) <= 5120 and (self.predictions['closest_enemy'] > 5120 or self.foe_goal.location.dist(self.me.location) < self.predictions['closest_enemy'] + 250)) or self.foe_goal.location.dist(self.ball.location) < 750:
                    self.line(*self.best_shot, self.renderer.team_color(alt_color=True))
                    shot = self.get_shot(self.best_shot, cap=2)

                    if shot is not None:
                        self.shoot_from(shot, clear_on_valid=True)
                elif self.can_shoot is None:
                    if self.ball.location.y * side(self.team) > -1280:
                        if self.me.location.y * side(self.team) > self.ball.location.y * side(self.team) and self.smart_shot(cap=1.5):
                            return

                        for i, shot in enumerate(self.defensive_shots[1:]):
                            shot = self.get_shot(shot, cap=1.5)

                            if shot is not None:
                                self.shoot_from(shot)
                                return

                    shots = [self.offensive_shots[0]]
                    if self.ball.location.x * side(not self.team) > 1000:
                        shots.append(self.offensive_shots[1])
                    elif self.ball.location.x * side(not self.team) < -1000:
                        shots.append(self.offensive_shots[2])

                    for o_shot in shots:
                        self.line(*o_shot, self.renderer.team_color(alt_color=True))

                    for i, o_shot in enumerate(shots):
                        shot = self.get_shot(self.best_shot, cap=2) if i == 0 else None

                        if shot is None:
                            shot = self.get_shot(o_shot, cap=2)

                        if shot is not None:
                            self.shoot_from(shot, defend=False, clear_on_valid=True)
                            return

            if self.is_clear():
                if len(self.friends) > 1 and (abs(self.ball.location.y) > 2560 or self.predictions['self_to_ball'] > 1000) and self.can_shoot is None:
                    self.push(short_shot(self.foe_goal.location))
                else:
                    self.backcheck()
        elif self.odd_tick % 2 == 0 and self.shooting and self.can_shoot is None:
            if self.predictions['goal'] or (self.foe_goal.location.dist(self.ball.location) <= 1500 and (self.predictions['closest_enemy'] > 1400 or self.foe_goal.location.dist(self.me.location) < self.predictions['closest_enemy'] + 250)):
                self.line(*self.best_shot, self.renderer.team_color(alt_color=True))
                shot = self.get_shot(self.best_shot)

                if shot is not None:
                    self.shoot_from(shot, clear_on_valid=True)
            elif self.odd_tick == 0:
                if self.ball.location.y * side(self.team) > -1280:
                    if self.me.location.y * side(self.team) > self.ball.location.y * side(self.team) and self.smart_shot():
                        return

                    for i, shot in enumerate(self.defensive_shots[1:]):
                        shot_weight = get_weight(self, index=i)

                        if shot_weight < self.shot_weight:
                            break

                        shot = self.get_shot(shot)

                        if shot is not None:
                            self.shoot_from(shot, clear_on_valid=True)
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
                            self.shoot_from(shot, clear_on_valid=True)
                            return

        if (self.is_clear() or not self.shooting) and self.odd_tick == 0:
            if not self.smart_shot(self.best_shot, cap=2 if self.can_shoot is None else 1) and (self.can_shoot is not None or not self.smart_shot(self.offensive_shots[0], cap=2.5)) and self.is_clear() and self.ball.location.y * side(self.team) > self.me.location.y - 250:
                self.backcheck()

    def attack_air(self):
        if self.odd_tick % 2 == 0 and self.can_shoot is None and self.me.boost > 12 and not self.shooting:
            if self.predictions['goal'] or (self.foe_goal.location.dist(self.ball.location) <= 1500 and (self.predictions['closest_enemy'] > 1400 or self.foe_goal.location.dist(self.me.location) < self.predictions['closest_enemy'] + 250)):
                self.line(*self.best_shot, self.renderer.team_color(alt_color=True))
                shot = self.get_shot(self.best_shot, cap=2)

                if shot is not None:
                    self.shoot_from(shot, clear_on_valid=True)
            elif self.odd_tick == 0:
                self.line(*self.offensive_shots[0], self.renderer.team_color(alt_color=True))

                shot = self.get_shot(self.best_shot, weight=self.max_shot_weight + 1, cap=2)

                if shot is None:
                    shot = self.get_shot(self.offensive_shots[0], weight=self.max_shot_weight, cap=2)

                if shot is not None:
                    self.shoot_from(shot, clear_on_valid=True)

    def get_shot(self, target=None, weight=None, cap=None):
        if self.can_shoot is None:
            jump_shot = aerial_shot = None

            if weight is None:
                weight = get_weight(self, target)

            if target is not None:
                if not self.me.airborne and not self.air_bud:
                    jump_shot = find_jump_shot(self, target, weight=weight, cap_=6 if cap is None else cap)

                aerial_shot = find_aerial(self, target, weight=weight, cap_=4 if cap is None or cap > 4 else cap)
            else:
                if not self.me.airborne and not self.air_bud:
                    jump_shot = find_any_jump_shot(self, cap_=6 if cap is None else cap)

                aerial_shot = find_any_aerial(self, cap_=4 if cap is None or cap > 4 else cap)

            shots = []

            if aerial_shot is not None:
                shots.append(aerial_shot)

            if jump_shot is not None:
                shots.append(jump_shot)

            if len(shots) == 0:
                return

            if len(shots) == 2:
                shots.sort(key=lambda shot: shot.intercept_time)

            return {
                "weight": weight,
                "intercept_time": shots[0].intercept_time,
                "shot": shots[0]
            }

    def shoot_from(self, shot, defend=True, clear_on_valid=False):
        if self.can_shoot is None or self.predictions['own_goal']:
            if (defend and not self.shooting and not self.is_clear()) or clear_on_valid:
                self.clear()

            if self.is_clear():
                self.shooting = True
                self.shot_time = shot['intercept_time']
                self.shot_weight = shot['weight']

                self.push(shot['shot'])
                self.send_quick_chat(QuickChats.CHAT_TEAM_ONLY, QuickChats.Information_IGotIt)

    def backcheck(self, simple=False, clear_on_valid=False):
        if self.is_clear() or clear_on_valid:
            ball_slice = self.predictions['ball_struct'].slices[self.future_ball_location_slice].physics.location
            ball = Vector(ball_slice.x, ball_slice.y, ball_slice.z)
            if self.playstyle is not self.playstyles.Defensive and not simple and ball.y * side(self.team) < (2560 if len(self.friends) > 0 else 0) and not self.predictions['own_goal']:
                if clear_on_valid:
                    self.clear()

                self.push(dynamic_backcheck())
                return True

            routine_retreat = retreat()
            if self.me.location.dist(routine_retreat.get_target(self)) > 350:
                if clear_on_valid:
                    self.clear()

                self.push(retreat())
                return True

        return False

    def defensive_kickoff(self):
        self.playstyle = self.playstyles.Defensive
        self.can_shoot = self.time

        self.print("Defending!")

        send_comm(self, {
            "match_defender": True
        })
        self.send_quick_chat(QuickChats.CHAT_TEAM_ONLY, QuickChats.Information_Defending)
        self.push(retreat())
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
            self.push(back_kickoff())
        elif kickoff_check(left):
            self.push(corner_kickoff(-1))
        elif kickoff_check(right):
            self.push(corner_kickoff(1))
        elif kickoff_check(back_left) or kickoff_check(back_right):
            self.push(generic_kickoff())
        else:
            self.print("Unknown kickoff position; skipping")
            self.kickoff_done = True
            return

        self.can_shoot = self.time

        self.send_quick_chat(QuickChats.CHAT_TEAM_ONLY, QuickChats.Information_IGotIt)

        self.print("I got it!")

        send_comm(self, {
            "attacking": True
        })

        self.playstyle = self.playstyles.Offensive

    def goto_nearest_boost(self, only_small=False):
        if self.is_clear():
            self.send_quick_chat(QuickChats.CHAT_TEAM_ONLY, QuickChats.Information_NeedBoost)

            if not only_small:
                large_boosts = (boost for boost in self.boosts if boost.large and boost.active and ((self.playstyle is self.playstyles.Offensive and boost.location.y * side(self.team) < -3000) or (self.playstyle is self.playstyles.Neutral and boost.location.y * side(self.team) > -100) or (self.playstyle is self.playstyles.Defensive and boost.location.y * side(self.team) > 3000)))

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
                closest_distance = closest.location.flat_dist(self.me.location)

                for item in small_boosts:
                    item_distance = item.location.flat_dist(self.me.location)

                    if item_distance < closest_distance:
                        item_loc = item.location.y * side(self.team)
                        if (self.playstyle is self.playstyles.Offensive and item_loc < -2560) or (self.playstyle is self.playstyles.Neutral and item_loc < -100) or (self.playstyle is self.playstyles.Defensive and item_loc > 1280):
                            closest = item
                            closest_distance = item_distance

                self.push(goto_boost(closest))
