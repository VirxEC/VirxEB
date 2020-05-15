from tools import *
from objects import *
from routines import *
from vec import *

from queue import Empty

class VirxEB(GoslingAgent):
    def run(self):
        for i in range(8):
            try:
                msg = self.matchcomms.incoming_broadcast.get_nowait()
            except Empty:
                break
            
            if len(self.stack) < 1:
                if msg.get("VirxEB") is not None:
                    if msg["VirxEB"].get("attacking") == True:
                        print("VirxEB: Defending!")
                        self.push(goto(self.friend_goal.location))
                        # foe_distances = [Vec3(foe.location).dist(self.me.location) for foe in self.foes]
                        # nearest_foe_index = foe_distances.index(min(foe_distances))
                        # self.push(demoDefence(nearest_foe_index))
                    elif msg["VirxEB"].get("centering") == True:
                        if not self.defender:
                            print("VirxeC: Getting ready for the pass!")
                            self.push(goto(Vector3(0,0,0)))
        
        if self.defender:
            if self.kickoff_flag:
                friend_distances = [Vec3(friend.location).dist(Vec3(self.ball.location)) for friend in self.friends]
                friend_distances.append(Vec3(self.me.location).dist(Vec3(self.ball.location)))
                min_distance = min(friend_distances)
                max_distance = max(friend_distances)

                car_distance = Vec3(self.me.location).dist(Vec3(self.ball.location))

                if min_distance - 5 < car_distance and car_distance < min_distance + 5:
                    print("VirxEB: Going for the kickoff!")
                    self.push(kickoff())
                    self.matchcomms.outgoing_broadcast.put_nowait({
                        "VirxEB": {
                            "attacking": True
                        }
                    })
                    self.defender = False
                elif max_distance - 5 < car_distance and car_distance < max_distance + 5:
                    self.defender = True
                self.shooting = False
            elif self.me.airborne:
                self.shooting = False
                self.push(recovery(self.friend_goal.location))
            else:
                targets = {
                    "goal": (self.foe_goal.left_post, self.foe_goal.right_post)
                }
                shots = find_hits(self, targets)
                if len(shots['goal']) > 0:
                    if self.shooting == False:
                        while len(self.stack) != 0:
                            self.pop()
                    self.push(shots["goal"][0])
                    self.shooting = True
                
                if len(self.stack) < 1:
                    self.shooting = False
                    if self.me.boost < 24 and self.ball.latest_touched_team == self.team:
                        self.goto_nearest_boost(only_small=True)
                    
                    self.push(goto(self.friend_goal.location))

        elif len(self.stack) < 1:
            if self.kickoff_flag:
                friend_distances = [Vec3(friend.location).dist(Vec3(self.ball.location)) for friend in self.friends]
                friend_distances.append(Vec3(self.me.location).dist(Vec3(self.ball.location)))
                min_distance = min(friend_distances)
                max_distance = max(friend_distances)

                car_distance = Vec3(self.me.location).dist(Vec3(self.ball.location))

                if min_distance - 5 < car_distance and car_distance < min_distance + 5:
                    print("VirxEB: Going for the kickoff!")
                    self.push(kickoff())
                    self.matchcomms.outgoing_broadcast.put_nowait({
                        "VirxEB": {
                            "attacking": True
                        }
                    })
                    self.defender = False
                elif max_distance - 5 < car_distance and car_distance < max_distance + 5:
                    self.defender = True
            elif self.me.airborne:
                self.push(recovery(self.friend_goal.location))
            else:
                targets = {
                    "goal": (self.foe_goal.left_post, self.foe_goal.right_post)
                }
                shots = find_hits(self, targets)
                if len(shots['goal']) > 0:
                    self.push(shots["goal"][0])
                    self.matchcomms.outgoing_broadcast.put_nowait({
                        "VirxEB": {
                            "centering": True
                        }
                    })
                elif self.me.boost < 24 and self.ball.latest_touched_team == self.team:
                    self.goto_nearest_boost()
                else:
                    self.push(goto(self.friend_goal.location))
                    self.push(short_shot(self.foe_goal.location))
    
    def goto_nearest_boost(self, only_small=False):
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

            self.push(goto_boost(closest, self.ball.location))
        else:
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

                self.push(goto_boost(closest, self.ball.location))