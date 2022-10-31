from typing import Optional

from util.agent import Car as BaseCar
from util.agent import GameTickPacket


class Car(BaseCar):
    def __init__(self, index: int, packet: Optional[GameTickPacket]=None):
        super().__init__(index, packet)
        self.minimum_time_to_ball = 7
