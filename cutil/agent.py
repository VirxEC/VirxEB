from typing import Optional

from util.agent import Car, GameTickPacket


class Car(Car):
    def __init__(self, index: int, packet: Optional[GameTickPacket]=None):
        super().__init__(index, packet)
        self.minimum_time_to_ball = 7
