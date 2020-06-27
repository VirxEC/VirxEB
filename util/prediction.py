from queue import Full, Queue
from threading import Thread

from rlbot.utils.structures.quick_chats import QuickChats

from util.interface import set_prediction, get_predictions


class EnemyPrediction:
    def __init__(self):
        self.queue = Queue(3)
        worker = Thread(target=self.main, args=(self.queue,))
        worker.setDaemon(True)
        worker.start()

    def main(self, q):
        while True:
            agent = q.get()

            foe_distances = []

            for foe in agent['foes']:
                foe_dist = agent['ball'].location.dist(foe.location)
                foe_distances.append(foe_dist)

                if len(agent['friends']) == 0 and foe_dist < 500 and agent['ball_to_goal'] > 4000 and foe.location.y - 200 < agent['ball'].location.y and agent['ball'].location.y < foe.location.y + 200:
                    set_prediction('can_shoot', False)
                    break
                else:
                    set_prediction('can_shoot', True)

            set_prediction('closest_enemy', min(foe_distances))

            q.task_done()

    def add_agent(self, agent):
        try:
            self.queue.put_nowait(agent)
        except Full:
            print(f"VirxEB ({ agent['me'].index }): Enemy prediction is lagging behind...")
            self.queue.join()
            print(f"VirxEB ( { agent['me'].index }): All caught up! Here I go!")


class BallPrediction:
    def __init__(self):
        self.queue = Queue(3)
        worker = Thread(target=self.main, args=(self.queue,))
        worker.setDaemon(True)
        worker.start()

    def main(self, q):
        while True:
            agent = q.get()

            is_own_goal = False
            is_goal = False

            ball_prediction = agent['ball_struct']
            set_prediction('ball_struct', ball_prediction)

            if ball_prediction is not None:
                for i in range(0, ball_prediction.num_slices, 2):
                    prediction_slice = ball_prediction.slices[i]
                    location = prediction_slice.physics.location

                    if (agent['team'] == 0 and location.y <= -7680) or (agent['team'] == 1 and location.y >= 7680):
                        is_own_goal = True
                        break
                    elif (agent['team'] == 0 and location.y >= 7680) or (agent['team'] == 1 and location.y <= -7680):
                        is_goal = True
                        break

            if is_own_goal:
                if not get_predictions()['own_goal']:
                    agent.send_quick_chat(
                        QuickChats.CHAT_EVERYONE, QuickChats.Compliments_NiceShot)

            set_prediction("own_goal", is_own_goal)
            set_prediction("goal", is_goal)

            q.task_done()

    def add_agent(self, agent):
        try:
            self.queue.put_nowait(agent)
        except Full:
            print(f"VirxEB ({ agent['me'].index }): Ball prediction is lagging behind...")
            self.queue.join()
            print(f"VirxEB ( { agent['me'].index }): All caught up! Here I go!")


class TeammatePrediction:
    def __init__(self):
        self.queue = Queue(3)
        worker = Thread(target=self.main, args=(self.queue,))
        worker.setDaemon(True)
        worker.start()

    def main(self, q):
        while True:
            agent = q.get()

            teammates = agent['friends']
            teammates.append(agent['me'])

            teammates_from_goal = [agent['ball'].location.dist(teammate.location) for teammate in teammates]
            set_prediction("teammates_from_goal", teammates_from_goal)

            if agent['ball'].location.dist(agent['me'].location) == min(teammates_from_goal):
                set_prediction("can_shoot", False)
            else:
                set_prediction("can_shoot", True)

            q.task_done()

    def add_agent(self, agent):
        try:
            self.queue.put_nowait(agent)
        except Full:
            print(f"VirxEB ({ agent['me'].index }): Teammate prediction is lagging behind...")
            self.queue.join()
            print(f"VirxEB ( { agent['me'].index }): All caught up! Here I go!")
