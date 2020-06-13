from util.tools import *
from util.objects import *
from util.routines import *
from util.vec import *

from queue import Empty
from rlbot.utils.structures.quick_chats import QuickChats
from rlbot_action_server.models import BotAction, AvailableActions

# If you want to run tests:
# Remove GoslingAgent from VirxEB
# Add GoslingAgent to Test


class VirxEB(GoslingAgent):
    def get_actions_currently_available(self):
        actions = AvailableActions(entity_name="VirxEB", current_action=self.action_broker.current_action,
            available_actions=[
                BotAction(description="get large boost", action_type="get_boost", data={'value': True}),
                BotAction(description="get small boost", action_type="get_boost", data={'value': False}),
                BotAction(description="attack", action_type="playstyle", data={'value': False}),
                BotAction(description="defend", action_type="playstyle", data={'value': True}),
                BotAction(description="panic", action_type="panic", data={}),
                BotAction(description="backcheck", action_type="backcheck", data={}),
            ])
        return [actions]
    
    def init(self):
        team = 1 if self.team == 0 else -1

        self.defensive_shots = [
            (Vector3(3100, 0, 100), Vector3(2900, 0, 100)),
            (Vector3(-3100, 0, 100), Vector3(-2900, 0, 100)),
            (Vector3(3600, 0, 200), Vector3(-3600, 0, 200)),
            (Vector3(3100, team * 3250, 100), Vector3(2900, team * 3250, 100)),
            (Vector3(-3100, team * 3250, 100), Vector3(-2900, team * 3250, 100))
        ]

        self.defensive_shot = None

    def run(self):
        if self.kickoff_flag and self.is_clear():
            self.do_kickoff()

        self.handle_matchcomms()

        if self.defender:
            self.playstyle_defend()
        else:
            self.playstyle_attack()

        if self.is_clear():
            self.shooting = False

        if not self.shooting:
            if self.team == 1 and self.me.location.y > 5100:
                self.backcheck()
            elif self.team == 0 and self.me.location.y < -5100:
                self.backcheck()
        
        if self.action_broker.current_action:
            action_type = self.action_broker.current_action.action_type
            ca_data = self.action_broker.current_action.data['value']
            
            if action_type == "playstyle":
                self.defender = ca_data
            elif action_type == "panic":
                self.panic_at(50000, 50000)
            elif action_type == "get_boost":
                if not self.is_clear():
                    self.clear()
                
                self.goto_nearest_boost(only_small=ca_data)
            elif action_type == "backcheck":
                if not self.is_clear():
                    self.clear()
                
                self.backcheck()
        
        return self.controller
                    

    def smart_shot(self, shot):
        shot = self.get_shot(shot)
        if shot is not None:
            self.shoot_from(shot)
            return True
        return False

    def panic_at(self, far_panic, close_panic):
        if not self.shooting and self.ball_to_goal < far_panic:
            self.panic = True

            if not self.is_clear():
                self.clear()

            if self.defensive_shot == None:
                self.defensive_shot = 0

            if self.defensive_shot >= len(self.defensive_shots):
                self.defensive_shot = None
                if self.ball_to_goal < close_panic:
                    self.push(short_shot(Vector3(0, 0, 320)))
                return

            if self.smart_shot(self.defensive_shots[self.defensive_shot]):
                self.defensive_shot = None
                return

            self.defensive_shot += 1
        else:
            self.panic = False

    def playstyle_defend(self):
        self.panic_at(5000, 2000)

        if not self.shooting:
            if self.is_clear():
                if self.me.airborne:
                    self.recover_from_air()
                elif self.me.boost < 36 and self.ball.latest_touched_team == self.team:
                    self.goto_nearest_boost(only_small=True)
                else:
                    self.backcheck()

    def playstyle_attack(self):
        self.panic_at(2500, 1000)

        if not self.shooting:
            if self.is_clear():
                if self.me.airborne:
                    self.recover_from_air()
                else:
                    shot = self.get_shot(
                        (self.foe_goal.left_post, self.foe_goal.right_post))
                    if shot is not None:
                        self.shoot_from(shot, defend=False)
                    elif self.me.boost < 24 and self.ball.latest_touched_team == self.team:
                        self.goto_nearest_boost()
                    else:
                        self.push(short_shot(self.foe_goal.location))
                        self.backcheck()

    def get_shot(self, target):
        shots = (find_hits(self, {"target": target}))['target']
        return None if len(shots) == 0 else shots[0]

    def handle_matchcomms(self):
        for i in range(32):
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
                                    f"VirxEB: Index ({ self.index }) is no longer a round defender")

                    else:
                        if self.is_clear():
                            if msg.get("attacking") == True:
                                print("VirxEB: Defending!")
                                self.send_quick_chat(
                                    QuickChats.CHAT_EVERYONE, QuickChats.Information_Defending)
                                self.push(goto(self.friend_goal.location,
                                               self.foe_goal.location))
                                # foe_distances = [Vec3(foe.location).dist(self.me.location) for foe in self.foes]
                                # nearest_foe_index = foe_distances.index(min(foe_distances))
                                # self.push(demoDefence(nearest_foe_index))
                            elif msg.get("centering") == True:
                                print("VirxEB: Getting ready for the pass!")
                                self.push(goto(Vector3(0, 0, 0)))

    def shoot_from(self, shot, defend=True):
        if defend and not self.shooting and not self.is_clear():
            self.clear()

        self.push(shot)

        if defend:
            self.send_quick_chat(QuickChats.CHAT_EVERYONE,
                                 QuickChats.Information_IGotIt)
        else:
            if self.ball_to_goal > 5000:
                self.send_comm({
                    "centering": True
                })
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

    def backcheck(self):
        if (self.friend_goal.location - self.me.location).flatten().magnitude() > 200:
            self.push(goto(self.friend_goal.location, self.ball.location))

    def recover_from_air(self):
        self.shooting = False
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
                shot = self.get_shot(
                    (self.foe_goal.right_post, self.foe_goal.left_post))

                if shot is None:
                    self.push(kickoff())
                else:
                    self.shoot_from(shot)

                self.send_quick_chat(QuickChats.CHAT_EVERYONE,
                                     QuickChats.Information_IGotIt)
                self.send_comm({
                    "attacking": True
                })
                self.defender = False
        elif max_distance - 5 < car_distance and car_distance < max_distance + 5:
            self.shooting = False
            self.clear()

            self.defender = True
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
                return

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


class Test():
    def init(self):
        self.debugging = True

    def run(self):
        self.dbg_val(self.team)
        self.dbg_val(self.foes[0].location)