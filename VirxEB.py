from queue import Empty
from traceback import print_exc

# from rlbot.utils.game_state_util import BallState, GameState, Physics
# from rlbot.utils.game_state_util import Vector3
from rlbot.utils.structures.quick_chats import QuickChats

from util.objects import GoslingAgent, Vector
from util.routines import goto, goto_boost, recovery, short_shot, generic_kickoff, dynamic_backcheck, retreat, block_ground_shot
# from util.replays import back_left_kickoff, back_right_kickoff
from util.replays import back_kickoff, right_kickoff, left_kickoff
from util.tools import find_hits, find_risky_hits, find_aerial
from util.utils import side, sign, almost_equals, send_comm, get_weight, peek_generator


class VirxEB(GoslingAgent):
    def init(self):
        self.playstyles_switch = {
            self.playstyles.Defensive: self.playstyle_defend,
            self.playstyles.Offensive: self.playstyle_attack,
            self.playstyles.Neutral: self.playstyle_neutral
        }

    def test(self):
        # This is for ground shot blocking tests
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
        # This is for backcheck testing
        """
        self.dbg_2d(self.ball.location)
        self.dbg_2d(self.team)

        if self.is_clear():
            self.backcheck()

        """
        # This is for state setting the ball to high up for aerial testing
        """
        if not self.shooting and self.ball.location.z < 98:
            ball_state = BallState(Physics(location=Vector3(0, -3000 * side(self.team), self.ball.location.z), velocity=Vector3(0, 0, 2000), angular_velocity=Vector3(0, 0, 0)))
            game_state = GameState(ball=ball_state)
            self.set_game_state(game_state)

        if not self.shooting:
            self.smart_shot((self.friend_goal.right_post, self.friend_goal.left_post))

        if self.is_clear():
            self.push(goto(Vector(), self.foe_goal.location, brake=True))
        """

    def run(self):
        for _ in range(len(self.friends) + len(self.foes) + 1):
            try:
                msg = self.matchcomms.incoming_broadcast.get_nowait()
            except Empty:
                break

            if msg.get("VirxEB") is not None and msg['VirxEB']['team'] is self.team:
                msg = msg['VirxEB']
                if self.playstyle is self.playstyles.Defensive:
                    if msg.get("match_defender") and msg['index'] < self.index:
                        self.playstyle = self.playstyles.Neutral
                        self.clear()
                        self.goto_nearest_boost()

                        self.print("You can defend")
                elif self.playstyle is self.playstyles.Offensive:
                    if msg.get("attacking") and msg['index'] < self.index:
                        self.playstyle = self.playstyles.Neutral
                        self.clear()
                        self.goto_nearest_boost()
                        self.kickoff_done = True
                        self.can_shoot = self.time

                        self.print("All yours!")

        if not self.kickoff_done:
            if self.is_clear():
                if len(self.friends) > 0:
                    try:
                        if almost_equals(min(self.predictions['team_to_ball']), self.predictions['self_to_ball'], 5):
                            self.offensive_kickoff()
                        elif almost_equals(max(self.predictions['team_to_ball']), self.predictions['self_to_ball'], 5):
                            self.defensive_kickoff()
                    except ValueError:
                        return
                elif almost_equals(self.predictions['closest_enemy'], self.predictions['self_to_ball'], 50):
                    self.offensive_kickoff()
                else:
                    self.defensive_kickoff()
        else:
            if self.can_shoot is not None and self.time - self.can_shoot >= 3:
                self.can_shoot = None

            # This should help VirxEB get out of the net, and also not patch 'guarding the goal posts'
            if not self.is_clear() and self.shooting and self.shot_weight == -1 and (self.predictions['closest_enemy'] < 1000 or self.ball_to_goal > 1500):
                self.clear()
            elif self.is_clear() and self.predictions['closest_enemy'] is not None and self.predictions['closest_enemy'] > 2500 and self.ball_to_goal < 1500 and side(self.team) is sign(self.me.location.y) and abs(self.me.location.y) > 5225:
                self.push(short_shot(self.foe_goal.location))

            self.playstyles_switch[self.playstyle]()
        ""

    def handle_quick_chat(self, index, team, quick_chat):
        try:
            if team is self.team and index is not self.index:
                if quick_chat is QuickChats.Information_IGotIt:
                    self.can_shoot = self.time + 1
        except Exception:
            print_exc()

    def smart_shot(self, shot, weight=None, cap=None):
        shot = self.get_shot(shot, weight=weight, cap=cap)
        if shot is not None:
            self.clear()
            self.shoot_from(shot)
            return True
        return False

    def handle_panic(self):
        if side(self.team) * self.ball.location.y > 750 or self.predictions['own_goal']:
            self.panic = True

            for shots in (self.defensive_shots, self.panic_shots):
                for shot in shots:
                    self.line(*shot, self.renderer.team_color(alt_color=True))

            if not self.shooting:
                # What is 174.9?
                # 174.9 is the radius of the ball rounded up (93) plus the half the length of the longest car rounded up (breakout; 66) with an extra 10%
                # Basicly it's the 'is an enemy on top of the damn ball' detector
                if abs(self.me.location.y) > abs(self.ball.location.y) and len(self.friends) > 0 and min(self.predictions['team_to_ball']) != self.predictions['self_to_ball'] and self.predictions['closest_enemy'] < 174.9:
                    bgs = block_ground_shot()
                    if bgs.is_viable(self):
                        self.push(bgs)
                        return

                if (side(self.team) * self.ball.location.y > 4200 or self.predictions['own_goal']) and self.can_shoot is not None:
                    self.can_shoot = None

                for shot in self.defensive_shots:
                    if self.smart_shot(shot, cap=4):
                        return

                if side(self.team) * self.ball.location.y > 4000:
                    self.smart_shot((self.friend_goal.right_post, self.friend_goal.left_post), weight=0, cap=2)

            if (not self.shooting or self.shot_weight == -1) and self.odd_tick % 2 == 0:
                if abs(self.me.location.y) > abs(self.ball.location.y) and len(self.friends) > 0 and min(self.predictions['team_to_ball']) != self.predictions['self_to_ball']:
                    bgs = block_ground_shot()
                    if (self.predictions['own_goal'] or (len(self.friends) > 0 and max(self.predictions['team_to_ball']) == self.predictions['self_to_ball'])) and bgs.is_viable(self):
                        self.clear()
                        self.push(bgs)
                        return
                    if not self.smart_shot((self.friend_goal.right_post, self.friend_goal.left_post), weight=1, cap=2) and self.ball.location.z < 250:
                        for shot in self.panic_shots:
                            if self.smart_shot(shot, weight=1, cap=3):
                                return
                        self.clear()
                        self.push(short_shot(Vector(z=320)))
                elif (self.is_clear() or self.stack[-1].__class__.__name__ not in {"goto", "flip", "brake", "recovery"}) and not self.shooting:
                    self.clear()
                    self.backcheck()
        else:
            self.panic = False

    def playstyle_defend(self):
        if self.is_clear() and self.me.airborne:
            self.recover_from_air()
        else:
            self.handle_panic()

            if not self.me.airborne:
                if self.shooting and not self.predictions['own_goal'] and self.ball.location.z * side(self.team) < 120:
                    self.clear()

                if self.is_clear():
                    if self.predictions['self_from_goal'] > 2560:
                        self.backcheck(simple=True)
                    if not self.panic and self.me.boost < 72 and self.ball.latest_touched_team is self.team and not self.panic:
                        self.goto_nearest_boost(only_small=True)
                    elif self.predictions['self_from_goal'] > 750:
                        self.backcheck(simple=True)

    def playstyle_neutral(self):
        if self.is_clear() and self.me.airborne:
            self.recover_from_air()
        else:
            self.handle_panic()

            if not self.me.airborne and self.is_clear() and not self.panic:
                if self.predictions['self_to_ball'] > 3413:
                    self.backcheck()
                elif self.me.boost < 60 and self.ball.location.y * side(self.team) < -2560 and not self.predictions['goal'] and self.ball.location.flat_dist(self.foe_goal.location) > 1280:
                    self.goto_nearest_boost()
                else:
                    self.backcheck()
            elif self.odd_tick % 2 == 0 and self.shooting and not self.me.airborne and self.can_shoot is None:
                shot = self.get_shot(self.best_shot)
                if shot is None:
                    shot = self.get_shot(self.offensive_shots[0])

                if shot is not None:
                    if shot['intercept_time'] < self.shot_time - 0.05:
                        self.clear()
                        self.shoot_from(shot)

            if self.is_clear() or self.stack[-1].__class__.__name__ in {"goto", "goto_boost", "brake", "dynamic_backcheck", "retreat"} and self.odd_tick == 0:
                if not self.smart_shot(self.best_shot) and not self.smart_shot(self.offensive_shots[0]) and self.is_clear() and not self.me.airborne:
                    self.backcheck()

    def playstyle_attack(self):
        if self.is_clear() and self.me.airborne:
            self.recover_from_air()
        else:
            self.handle_panic()

            if not self.me.airborne:
                if not self.shooting or self.shot_weight == -1:
                    if self.me.boost == 0:
                        self.clear()
                        self.backcheck()
                    else:
                        if self.predictions['goal'] or (self.foe_goal.location.dist(self.ball.location) <= 5120 and (self.predictions['closest_enemy'] > 5120 or self.foe_goal.location.dist(self.me.location) < self.predictions['closest_enemy'] + 250)) or self.foe_goal.location.dist(self.ball.location) < 750:
                            self.line(*self.best_shot, self.renderer.team_color(alt_color=True))
                            shot = self.get_shot(self.best_shot)

                            if shot is not None:
                                self.clear()
                                self.shoot_from(shot, defend=False)
                        elif self.can_shoot is None:
                            for o_shot in self.offensive_shots:
                                self.line(*o_shot, self.renderer.team_color(alt_color=True))

                            for i, o_shot in enumerate(self.offensive_shots):
                                shot = self.get_shot(self.best_shot) if i == 0 else None

                                if shot is None:
                                    shot = self.get_shot(o_shot)

                                if shot is not None:
                                    self.clear()
                                    self.shoot_from(shot, defend=False)

                    if self.is_clear():
                        if abs(self.ball.location.y) > 2560 or self.predictions['self_to_ball'] > 1000:
                            self.push(short_shot(self.foe_goal.location))
                        else:
                            self.backcheck()
                elif self.odd_tick % 2 == 0 and self.shooting:
                    if self.predictions['goal'] or (self.foe_goal.location.dist(self.ball.location) <= 1500 and (self.predictions['closest_enemy'] > 1400 or self.foe_goal.location.dist(self.me.location) < self.predictions['closest_enemy'] + 250)):
                        if self.odd_tick % 2 == 0:
                            self.line(*self.best_shot, self.renderer.team_color(alt_color=True))
                            shot = self.get_shot(self.best_shot)

                            if shot is not None:
                                if self.max_shot_weight is self.shot_weight:
                                    if shot['intercept_time'] < self.shot_time - 0.05:
                                        self.clear()
                                        self.shoot_from(shot)
                                elif shot['intercept_time'] <= min(self.shot_time + (self.max_shot_weight - self.shot_weight / 3), 5):
                                    self.clear()
                                    self.shoot_from(shot)
                    elif self.odd_tick == 0:
                        for o_shot in self.offensive_shots:
                            self.line(*o_shot, self.renderer.team_color(alt_color=True))

                        for i, o_shot in enumerate(self.offensive_shots):
                            shot = None
                            if i == 0:
                                shot_weight = self.max_shot_weight + 1
                                shot = self.get_shot(self.best_shot)

                            if shot is None:
                                shot_weight = get_weight(self, index=i)

                                if shot_weight < self.shot_weight:
                                    break

                                shot = self.get_shot(o_shot, weight=shot_weight)

                            if shot is not None:
                                if shot_weight is self.shot_weight:
                                    if shot['intercept_time'] < self.shot_time - 0.05:
                                        self.clear()
                                        self.shoot_from(shot)
                                elif shot['intercept_time'] <= min(self.shot_time + (shot_weight - self.shot_weight / 3), 5):
                                    self.clear()
                                    self.shoot_from(shot)

                if self.is_clear() or self.stack[-1].__class__.__name__ in {"goto", "goto_boost", "brake", "dynamic_backcheck", "retreat"} and self.odd_tick == 0:
                    if not self.smart_shot(self.best_shot) and not self.smart_shot(self.offensive_shots[0]) and self.is_clear() and not self.me.airborne:
                        if self.team == 1 and self.ball.location.y > self.me.location.y + 250:
                            self.backcheck()
                        elif self.team == 0 and self.ball.location.y < self.ball.location.y - 250:
                            self.backcheck()

    def get_shot(self, target, weight=None, cap=None):
        if self.can_shoot is None:
            final = shot = aerial_shot = None

            if self.me.airborne or self.air_bud:
                if self.me.boost < 24:
                    return

                aerial_shot = find_aerial(self, cap_=3 if cap is None or cap > 3 else cap) if target == (self.friend_goal.right_post, self.friend_goal.left_post) else find_risky_hits(self, target, cap_=4 if cap is None or cap > 4 else cap)
            else:
                shot = find_hits(self, target, cap_=6 if cap is None else cap)
                aerial_shot = find_risky_hits(self, target, cap_=4 if cap is None or cap > 4 else cap) if self.me.boost > 24 else None

            if shot is not None:
                final = aerial_shot if aerial_shot is not None and aerial_shot.intercept_time <= shot.intercept_time else shot
            elif aerial_shot is not None:
                final = aerial_shot

            if final is None:
                return

            return {
                "weight": get_weight(self, target) if weight is None else weight,
                "intercept_time": final.intercept_time,
                "shot": final
            }

        return

    def shoot_from(self, shot, defend=True):
        if defend and not self.shooting and not self.is_clear():
            self.clear()

        if self.is_clear() and self.can_shoot is None:
            self.shooting = True
            self.shot_time = shot['intercept_time']
            self.shot_weight = shot['weight']

            self.push(shot['shot'])
            self.send_quick_chat(QuickChats.CHAT_TEAM_ONLY, QuickChats.Information_IGotIt)

    def backcheck(self, simple=False):
        if self.is_clear():
            if self.playstyle != self.playstyles.Defensive and not simple and self.ball.location.y * side(not self.team) > -2560:
                self.push(dynamic_backcheck())
            elif self.me.location.dist(self.friend_goal.location + Vector(y=-250 * side(self.team))) > 500:
                self.push(retreat())
            else:
                return False

            return True

        return True

    def recover_from_air(self):
        self.clear()
        self.push(recovery(self.friend_goal.location))

    def defensive_kickoff(self):
        self.playstyle = self.playstyles.Defensive
        self.can_shoot = self.time

        self.print("Defending!")

        send_comm(self, {
            "match_defender": True
        })
        self.send_quick_chat(QuickChats.CHAT_TEAM_ONLY, QuickChats.Information_Defending)
        self.push(goto(self.friend_goal.location, self.foe_goal.location))
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
            self.push(left_kickoff())
        elif kickoff_check(right):
            self.push(right_kickoff())
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

        self.playstyle = self.playstyles.Offensive

    def goto_nearest_boost(self, only_small=False):
        if self.is_clear():
            self.send_quick_chat(QuickChats.CHAT_TEAM_ONLY, QuickChats.Information_NeedBoost)

            if not only_small:
                large_boosts = (boost for boost in self.boosts if boost.large and boost.active and self.friend_goal.location.y * side(self.team) - boost.location.y * side(self.team) >= 0)

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

                    if item_distance < closest_distance and item.location.flat_dist(self.friend_goal.location) < self.ball_to_goal - 750:
                        closest = item
                        closest_distance = item_distance

                if closest.location.flat_dist(self.friend_goal.location) < self.ball_to_goal - 750:
                    self.push(goto_boost(closest, self.ball.location))
