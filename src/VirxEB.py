from tools import *
from objects import *
from routines import *
from vec import *

from rlbot.utils.structures.quick_chats import QuickChats

class VirxEB(GoslingAgent):
    def run(self):
        if len(self.stack) < 1:
            if self.kickoff_flag:
                friend_distances = [Vec3(friend.location).dist(Vec3(self.ball.location)) for friend in self.friends]
                min_distance = min(friend_distances)
                if min_distance - 5 < friend_distances[self.index] and friend_distances[self.index] < min_distance + 5:
                    self.send_quick_chat(QuickChats.CHAT_TEAM_ONLY, QuickChats.Information_IGotIt)
                    self.push(kickoff())
                
            else:
                targets = {
                    "goal": (self.foe_goal.left_post, self.foe_goal.right_post)
                }
                shots = find_hits(self, targets)
                if len(shots['goal']) > 0:
                    self.send_quick_chat(QuickChats.CHAT_TEAM_ONLY, QuickChats.Information_IGotIt)
                    self.push(shots["goal"][0])
                else:
                    self.push(recovery(self.friend_goal.location))

                    if self.me.boost < 30:
                        large_boosts = [
                            boost for boost in self.boosts if boost.large and boost.active]
                        
                        if len(large_boosts) > 0:
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

                    self.push(short_shot(self.foe_goal.location))

    # def handle_quick_chat(self, index, team, quick_chat):
    #     if team == self.team and quick_chat == QuickChats.Information_IGotIt:
    #         foe_distances = [Vec3(foe.location).dist(self.me.location) for foe in self.foes]
    #         nearest_foe_index = foe_distances.index(min(foe_distances))
    #         self.push(demoDefence(nearest_foe_index))