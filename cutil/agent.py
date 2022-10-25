import hashlib
import json
import os
from pathlib import Path
from typing import Optional

from util.agent import Car, GameTickPacket


class Car(Car):
    def __init__(self, index: int, packet: Optional[GameTickPacket]=None, profile=True):
        super().__init__(index, packet)

        self.minimum_time_to_ball = 7
        self.profile = {}

        if not profile or self.true_name is None:
            return

        uuid = hashlib.sha1(self.true_name.encode()).hexdigest()
        self.profile_path = Path(os.path.dirname(__file__)).parent.parent / 'Vfiles' / f'{uuid}.json'
        if self.profile_path.exists():
            with open(self.profile_path, "r") as f:
                while 1:
                    try:
                        self.profile = json.load(f)
                        break
                    except Exception as e:
                        print(f"Error in {self.true_name}'s profile: {e}")
                        continue
        else:
            # create the parent directory if it doesn't exist
            self.profile_path.parent.mkdir(parents=True, exist_ok=True)
            with open(self.profile_path, "w") as f:
                f.write("{}")
