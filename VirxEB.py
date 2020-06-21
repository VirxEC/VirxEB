from queue import Empty

from rlbot.utils.structures.quick_chats import QuickChats

from util.objects import GoslingAgent, Vector3
from util.routines import short_shot, goto, recovery, goto_boost, aerial_shot, kickoff
from util.tools import find_hits
from util.vec import Vec3


class VirxEB(GoslingAgent):
    def init(self):
        foe_team = 1 if self.team == 0 else -1
        team = -1 * foe_team

        self.defensive_shots = (
            (self.foe_goal.left_post, self.foe_goal.right_post),
            (Vector3(3100, foe_team * 3250, 100),
             Vector3(2900, foe_team * 3250, 100)),
            (Vector3(-3100, foe_team * 3250, 100),
             Vector3(-2900, foe_team * 3250, 100)),
            (Vector3(2900, 0, 200), Vector3(-2900, 0, 200)),
            (Vector3(-3600, 0, 100), Vector3(-2900, 0, 100)),
            (Vector3(3600, 0, 100), Vector3(2900, 0, 100)),
            (Vector3(3100, team * 3250, 100), Vector3(2900, team * 3250, 100)),
            (Vector3(-3100, team * 3250, 100), Vector3(-2900, team * 3250, 100)),
        )

        self.offensive_shots = (
            (self.foe_goal.left_post, self.foe_goal.right_post),
            (Vector3(foe_team * 893, foe_team * 5120, 100), Vector3(foe_team * 893, foe_team * 4720, 320)),
            (Vector3(-foe_team * 893, foe_team * 5120, 100), Vector3(-foe_team * 893, foe_team * 4720, 320))
        )

        self.defensive_shot = None
        self.debugging = True

    def run(self):
        if self.kickoff_flag and self.is_clear():
            self.do_kickoff()

        self.handle_matchcomms()

        if self.can_shoot != None and self.time - self.can_shoot >= 3:
            self.can_shoot = None

        if self.defender:
            self.playstyle_defend()
        else:
            self.playstyle_attack()
            
        if self.is_clear():
            self.clear()

            if not self.shooting:
                if self.team == 1 and self.me.location.y > 5100:
                    self.backcheck()
                elif self.team == 0 and self.me.location.y < -5100:
                    self.backcheck()

        if self.shooting and not self.shooting_short:
            self.dbg_val(self.stack[0].intercept_time - self.time)

    def handle_quick_chat(self, index, team, quick_chat):
        if team != self.team and index != self.index:
            if quick_chat == QuickChats.Information_IGotIt:
                self.can_shoot = self.time

    def smart_shot(self, shot):
        shot = self.get_shot(shot)
        if shot is not None:
            self.shoot_from(shot)
            return True
        return False

    def panic_at(self, far_panic, close_panic):
        if self.ball_to_goal < far_panic:
            for d_shot in self.defensive_shots:
                self.line(*d_shot, color=(255, 0, 0))

            if not self.shooting:
                self.panic = True

                if not self.is_clear():
                    self.clear()

                if self.defensive_shot == None:
                    self.defensive_shot = 0

                if self.defensive_shot >= len(self.defensive_shots):
                    self.defensive_shot = None
                    if self.ball_to_goal < close_panic:
                        if abs(self.me.location.y) > abs(self.ball.location.y):
                            self.push(short_shot(Vector3(0, 0, 320)))
                        else:
                            team = -1 if self.team == 0 else 1
                            self.push(goto(Vector3(0, self.ball.location.y + (team * 200), 0)))
                    return

                if self.can_shoot == None:
                    if self.smart_shot(self.defensive_shots[self.defensive_shot]):
                        if not self.stack[0].intercept_time - self.time > 5:
                            self.defensive_shot = None
                            return

                        self.clear()

                    self.defensive_shot += 1
            else:
                self.panic = False
        else:
            self.panic = False

        if not self.panic:
            self.defensive_shot = None

    def playstyle_defend(self):
        self.panic_at(5000, 1000)

        if not self.shooting:
            if self.is_clear():
                if self.me.airborne:
                    self.recover_from_air()
                elif self.me.boost < 50 and self.ball.latest_touched_team == self.team:
                    self.goto_nearest_boost(only_small=True)
                else:
                    self.backcheck()

    def playstyle_attack(self):
        self.panic_at(2500, 1500)

        method_name = None

        if not self.is_clear():
            method_name = self.stack[0].__class__.__name__

        if not self.shooting and (self.is_clear() or method_name == "goto"):
            if self.me.airborne and not method_name == "goto":
                self.recover_from_air()
            else:
                for o_shot in self.offensive_shots:
                    self.line(*o_shot)

                found_shot = False

                if self.can_shoot == None:
                    for o_shot in self.offensive_shots:
                        shot = self.get_shot(o_shot)
                        if shot != None and not shot.intercept_time - self.time > 6:
                            self.clear()
                            self.shoot_from(shot, defend=False)
                            found_shot = True

                if not found_shot and not method_name == "goto":
                    if self.me.boost < 36 and self.ball.latest_touched_team == self.team:
                        self.goto_nearest_boost()
                    elif self.me.boost < 50 and self.ball.latest_touched_team == self.team:
                        self.goto_nearest_boost(only_small=True)
                    else:
                        if not self.backcheck() and self.me.boost < 50:
                            self.goto_nearest_boost(only_small=True)

    def get_shot(self, target):
        shots = (find_hits(self, {"target": target}))['target']
        return None if len(shots) == 0 else shots[0]

    def handle_matchcomms(self):
        for _ in range(32):
            try:
                msg = self.matchcomms.incoming_broadcast.get_nowait()
            except Empty:
                break

            if msg.get("VirxEB") is not None:
                msg = msg['VirxEB']

                if msg['team'] == self.team:
                    if self.defender:
                        if msg.get("match_defender") == True:
                            if msg['index'] < self.index:
                                self.defender = False
                                self.clear()
                                self.goto_nearest_boost()
                                print(
                                    f"VirxEB ({self.index}): You can defend")
                    else:
                        if msg.get("attacking") == True:
                            if msg['index'] < self.index:
                                self.clear()
                                self.goto_nearest_boost()

                                print(f"VirxEB ({self.index}): All yours!")


    def shoot_from(self, shot, defend=True):
        if defend and not self.shooting and not self.is_clear():
            self.clear()

        self.push(shot)

        self.send_quick_chat(QuickChats.CHAT_EVERYONE,
                             QuickChats.Information_IGotIt)

    def send_comm(self, msg):
        message = {
            "index": self.index,
            "team": self.team
        }
        msg.update(message)
        self.matchcomms.outgoing_broadcast.put_nowait({
            "VirxEB": msg
        })

    def backcheck(self, simple=False):
        if (self.friend_goal.location - self.me.location).flatten().magnitude() > 200:
            if not self.defender and not simple and (self.team == 0 and self.ball.location.y > 2560) or (self.team == 1 and self.ball.location.y < -2560):
                bc_x = 0

                if self.ball.location.x > 2048:
                    bc_x = 2048
                elif self.ball.location.x < -2048:
                    bc_x = -2048

                self.push(goto(Vector3(bc_x, 0, 0)))
            else:
                defensive_position = self.friend_goal.location + (Vector3(100, 0 ,0) if self.team == 0 else Vector3(-100, 0, 0))
                self.push(goto(defensive_position, self.ball.location))
                self.push(goto(defensive_position))
            return True
        
        return False

    def recover_from_air(self):
        self.clear()
        self.push(recovery(self.friend_goal.location))

    def do_kickoff(self):
        friend_distances = [Vec3(friend.location).dist(
            Vec3(self.ball.location)) for friend in self.friends]
        friend_distances.append(
            Vec3(self.me.location).dist(Vec3(self.ball.location)))
        min_distance = min(friend_distances)
        max_distance = max(friend_distances)

        car_distance = Vec3(self.me.location).dist(Vec3(self.ball.location))

        if min_distance - 5 < car_distance and car_distance < min_distance + 5:
            if not self.shooting or self.shooting_short:
                self.clear()

                shot = self.get_shot(
                    (self.foe_goal.right_post, self.foe_goal.left_post))

                if shot is None or shot.intercept_time > 3:
                    self.push(kickoff())
                else:
                    self.shoot_from(shot)

                self.send_quick_chat(QuickChats.CHAT_EVERYONE,
                                     QuickChats.Information_IGotIt)

                print(f"VirxEB ({self.index}): I got it!")

                self.send_comm({
                    "attacking": True
                })
                self.defender = False
        elif max_distance - 5 < car_distance and car_distance < max_distance + 5:
            self.clear()

            self.defender = True

            print(f"VirxEB ({self.index}): Defending!")

            self.send_comm({
                "match_defender": True
            })
            self.send_quick_chat(QuickChats.CHAT_EVERYONE,
                                 QuickChats.Information_Defending)
            self.push(goto(self.friend_goal.location, self.foe_goal.location))
        

    def goto_nearest_boost(self, only_small=False):
        self.send_quick_chat(QuickChats.CHAT_EVERYONE,
                             QuickChats.Information_NeedBoost)
        large_boosts = [
            boost for boost in self.boosts if boost.large and boost.active]

        if len(large_boosts) > 0 and only_small == False:
            closest = large_boosts[0]
            closest_distance = (
                large_boosts[0].location - self.me.location).magnitude()

            for item in large_boosts:
                item_disatance = (
                    item.location - self.me.location).magnitude()
                if item_disatance < closest_distance:
                    closest = item
                    closest_distance = item_disatance

            if closest_distance < 2500:
                self.push(goto_boost(closest, self.ball.location))

        small_boosts = [
            boost for boost in self.boosts if not boost.large and boost.active]

        if len(small_boosts) > 0:
            closest = small_boosts[0]
            closest_distance = (
                small_boosts[0].location - self.me.location).magnitude()

            for item in small_boosts:
                item_distance = (
                    item.location - self.me.location).magnitude()
                if item_distance < closest_distance:
                    closest = item
                    closest_distance = item_distance

            if closest_distance < 1000:
                self.push(goto_boost(closest, self.ball.location))