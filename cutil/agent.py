import hashlib
import json
import os
from typing import Optional

from util.agent import Car, GameTickPacket, VirxERLU


class Car(Car):
    def __init__(self, index: int, packet: Optional[GameTickPacket]=None, profile=True):
        super().__init__(index, packet)

        self.minimum_time_to_ball = 7
        self.profile = {}

        if not profile:
            return

        uuid = hashlib.sha1(self.true_name.encode()).hexdigest()
        self.profile_path = os.path.dirname(__file__) + f'/../../Vfiles/{uuid}.cpf'
        if os.path.isfile(self.profile_path):
            with open(self.profile_path, "r") as f:
                while 1:
                    try:
                        self.profile = json.load(f)
                        break
                    except Exception as e:
                        print(f"Error in {self.true_name}'s profile: {e}")
                        continue
        else:
            self.profile = {}
            with open(self.profile_path, "w") as f:
                json.dump(self.profile, f)
