from threading import Thread, Event

from rlbot.utils.structures.quick_chats import QuickChats


class Prediction(Thread):
    def __init__(self, agent):
        super().__init__(daemon=True)
        self.agent = agent
        self.event = Event()

    def run(self):
        while True:
            self.event.wait()

            len_friends = len(self.agent.friends)

            if len(self.agent.foes) > 0:
                foe_distances = []

                shoot_threshold = 4000

                if len_friends == 1:
                    shoot_threshold = 3500
                elif len_friends == 2:
                    shoot_threshold = 3000
                elif len_friends > 2:
                    shoot_threshold = 2750

                for foe in self.agent.foes:
                    foe_dist = self.agent.ball.location.dist(foe.location)
                    foe_distances.append(foe_dist)

                    if len_friends == 0 and foe_dist < 500 and self.agent.ball_to_goal > shoot_threshold and foe.location.y - 200 < self.agent.ball.location.y and self.agent.ball.location.y < foe.location.y + 200:
                        self.agent.predictions['can_shoot'] = False
                    else:
                        self.agent.predictions['can_shoot'] = True

                self.agent.predictions['closest_enemy'] = min(foe_distances)

            if len_friends > 0:
                teammates = self.agent.friends
                teammates.append(self.agent.me)

                teammates_from_goal = [self.agent.ball.location.dist(teammate.location) for teammate in teammates]
                self.agent.predictions["teammates_from_goal"] = teammates_from_goal

                if self.agent.ball.location.dist(self.agent.me.location) == min(teammates_from_goal):
                    self.agent.predictions["can_shoot"] = False
                else:
                    self.agent.predictions["can_shoot"] = True

            is_own_goal = False
            is_goal = False

            ball_prediction = self.agent.get_ball_prediction_struct()
            self.agent.predictions['ball_struct'] = ball_prediction

            if ball_prediction is not None:
                for i in range(0, ball_prediction.num_slices, 2):
                    prediction_slice = ball_prediction.slices[i]
                    location = prediction_slice.physics.location

                    if (self.agent.team == 0 and location.y <= -7680) or (self.agent.team == 1 and location.y >= 7680):
                        is_own_goal = True
                    elif (self.agent.team == 0 and location.y >= 7680) or (self.agent.team == 1 and location.y <= -7680):
                        is_goal = True

            if is_own_goal:
                if not self.agent.predictions['own_goal']:
                    self.agent.send_quick_chat(
                        QuickChats.CHAT_EVERYONE, QuickChats.Compliments_NiceShot)

            self.agent.predictions["own_goal"] = is_own_goal
            self.agent.predictions["goal"] = is_goal

            self.event.clear()
