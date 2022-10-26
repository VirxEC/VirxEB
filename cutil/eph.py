from __future__ import annotations

import json
import math
import os
import re
from copy import deepcopy
from hashlib import sha1
from pathlib import Path
from typing import Iterator, List

import numpy as np
from rlbot.utils.structures.ball_prediction_struct import BallPrediction
from rlbot.utils.structures.game_data_struct import (GameTickPacket,
                                                     PlayerInfo, Touch,
                                                     Vector3)
from util.utils import _fcap


class CarHeuristic(list):
    NAMES = (
        "may_ground_shot",
        "may_jump_shot",
        "may_double_jump_shot",
        "may_aerial"
    )

    def __init__(self):
        self.profile = [0.9, 0.9, 0.9, 0.9]

    def __str__(self):
        return str(self.profile)

    __repr__ = __str__

    def __iter__(self) -> Iterator:
        return iter(self.profile)

    def __len__(self) -> int:
        return len(self.profile)

    def __getitem__(self, index) -> float:
        return self.profile[index]

    def __setitem__(self, index, value):
        self.profile[index] = value


class PacketHeuristics:
    def __init__(self, threshold: float=0.8, gain: float=0.21, loss: float=0.0045, unpause_delay: float=0.75, ignore_indexes: List[int]=[], verbose: bool=False):
        self.cars = {}
        self.car_tracker = {}

        self.threshold = threshold
        self.gain = gain + loss
        self.loss = loss

        self.verbose = verbose

        self.ignore_indexes = ignore_indexes
        self.start_time = -1
        self.time = 0
        self.last_ball_touch_time = -1
        self.unpause_delay = unpause_delay
        self.last_pause_time = -1
        self.team_count = [0, 0]

        field_half_width = 4096
        field_third_width = field_half_width / 3

        field_half_length = 5120
        field_third_length = field_half_length / 3
        
        # the following comments are from the perspective of this diagram -> https://github.com/RLBot/RLBot/wiki/Useful-Game-Values
        self.zones = (
            (
                None,
                Zone2D(np.array((-field_third_width, 5120, 20)), np.array((field_third_width, 6000, 20))),  # orange net
                None,
            ),
            (
                Zone2D(np.array((field_third_width, field_third_length, 20)), np.array((field_half_width, field_half_length, 20))),  # orange field left
                Zone2D(np.array((-field_third_width, field_third_length, 20)), np.array((field_third_width, field_half_length, 20))),  # orange field
                Zone2D(np.array((-field_half_width, field_third_length, 20)), np.array((-field_third_width, field_half_length, 20))),  # orange field right
            ),
            (
                Zone2D(np.array((field_third_width, -field_third_length, 20)), np.array((field_half_width, field_third_length, 20))),  # mid field left
                Zone2D(np.array((-field_third_width, -field_third_length, 20)), np.array((field_third_width, field_third_length, 20))),  # mid field
                Zone2D(np.array((-field_half_width, -field_third_length, 20)), np.array((-field_third_width, field_third_length, 20))),  # mid field right
            ),
            (
                Zone2D(np.array((field_third_width, -field_half_length, 20)), np.array((field_half_width, -field_third_length, 20))), # blue field left
                Zone2D(np.array((-field_third_width, -field_half_length, 20)), np.array((field_third_width, -field_third_length, 20))),  # blue field
                Zone2D(np.array((-field_half_width, -field_half_length, 20)), np.array((-field_third_width, -field_third_length, 20))),  # blue field right
            ),
            (
                None,
                Zone2D(np.array((-field_third_width, -6000, 20)), np.array((field_third_width, -5120, 20))),  # blue net
                None,
            )
        )

        self.field_dimensions = [len(self.zones), len(self.zones[0])]

    def load_profile(true_car_name: str) -> dict:
        uuid = sha1(true_car_name.encode()).hexdigest()
        profile_path = Path(os.path.dirname(__file__)).parent.parent / 'Vfiles' / f'{uuid}.json'
        if profile_path.exists():
            with open(profile_path, "r") as f:
                try:
                    return json.load(f)
                except Exception as e:
                    print(f"Error when loading {true_car_name}'s profile: {e}")
        else:
            # create the parent directory if it doesn't exist
            profile_path.parent.mkdir(parents=True, exist_ok=True)
            with open(profile_path, "w") as f:
                f.write("{}")

        return {}

    def save_profiles(self):
        # the key is true_car_name
        # the value is the json we want to write to disk
        for true_car_name, profile in self.cars.items():
            uuid = sha1(true_car_name.encode()).hexdigest()
            profile_path = Path(os.path.dirname(__file__)).parent.parent / 'Vfiles' / f'{uuid}.json'
            with open(profile_path, "w") as f:
                json.dump(profile, f)

    def add_tick(self, packet: GameTickPacket, ball_prediction_struct: BallPrediction) -> bool:
        time = packet.game_info.seconds_elapsed
        delta_time = time - self.time
        self.time = time

        if self.start_time == -1:
            self.start_time = self.time

        if not packet.game_info.is_round_active or packet.game_info.is_kickoff_pause:
            self.last_pause_time = self.time
            if not packet.game_info.is_round_active:
                return False

        team_count = [0, 0]

        for i in range(packet.num_cars):
            team_count[packet.game_cars[i].team] += 1

        self.team_count = team_count

        loss = self.loss * delta_time

        latest_touch = packet.game_ball.latest_touch
        handled_touch = latest_touch.time_seconds <= self.last_ball_touch_time
        self.last_ball_touch_time = latest_touch.time_seconds
        
        ball_zone_id = self.get_zone_id(packet.game_ball.physics.location)

        future_ball_zones = {
            ball_zone_id: set((get_hashable_from_vector3(packet.game_ball.physics.location),))
        }

        for slice_ in ball_prediction_struct.slices[::15]:
            ball_location = slice_.physics.location
            future_ball_zone_id = self.get_zone_id(ball_location)

            if future_ball_zone_id not in future_ball_zones:
                future_ball_zones[future_ball_zone_id] = set()

            future_ball_zones[future_ball_zone_id].add(get_hashable_from_vector3(ball_location))

        for i in range(packet.num_cars):
            if i in self.ignore_indexes:
                continue

            car = packet.game_cars[i]
            if car.is_demolished:
                continue

            true_car_name = self.get_true_car_name(car.name)

            if car.name not in self.car_tracker:
                self.car_tracker[car.name] = {
                    "last_wheel_contact": {
                        "time": -1,
                        "up": np.zeros(3),
                        "location": np.zeros(3),
                    },
                    "zone_id": -1,
                    "friends": -1,
                    "foes": -1
                }

            if true_car_name not in self.cars:
                self.cars[true_car_name] = PacketHeuristics.load_profile(true_car_name)

            friends = self.car_tracker[car.name]['friends'] = self.get_friend_count(car.team)
            foes = self.car_tracker[car.name]['foes'] = self.get_foe_count(car.team)

            if friends not in self.cars[true_car_name]:
                self.cars[true_car_name][friends] = {}

            if foes not in self.cars[true_car_name][friends]:
                self.cars[true_car_name][friends][foes] = {}

            zone_id = self.get_zone_id(car.physics.location)

            if zone_id is None and self.verbose:
                print(f"WARNING: zone_id for {car.name} was None")
                continue

            self.car_tracker[car.name]['zone_id'] = zone_id

            if len(self.cars[true_car_name][friends][foes]) == 0:
                self.cars[true_car_name][friends][foes] = {i: CarHeuristic() for i in range(self.field_dimensions[0] * self.field_dimensions[1])}
            elif zone_id not in self.cars[true_car_name][friends][foes]:
                self.cars[true_car_name][friends][foes][zone_id] = CarHeuristic()

            if car.has_wheel_contact:
                self.car_tracker[car.name]['last_wheel_contact']['time'] = self.time
                self.car_tracker[car.name]['last_wheel_contact']['location'] = get_np_from_vector3(car.physics.location)
                CP = math.cos(car.physics.rotation.pitch)
                SP = math.sin(car.physics.rotation.pitch)
                CY = math.cos(car.physics.rotation.yaw)
                SY = math.sin(car.physics.rotation.yaw)
                CR = math.cos(car.physics.rotation.roll)
                SR = math.sin(car.physics.rotation.roll)
                self.car_tracker[car.name]['last_wheel_contact']['up'] = np.array((-CR*CY*SP-SR*SY, -CR*SY*SP+SR*CY, CP*CR))

            if packet.game_info.is_kickoff_pause or self.time - self.last_pause_time < self.unpause_delay:
                continue

            # Ball heuristic
            surrounding_zone_ids = self.get_surrounding_zone_ids(zone_id)

            if ball_zone_id in surrounding_zone_ids:
                if friends > 0:
                    car_loss = loss / ((friends + 1) / 1.6)
                else:
                    car_loss = loss / 0.8

                if not handled_touch and latest_touch.player_index == i and latest_touch.time_seconds > self.start_time:
                    ball_touch_section = self.get_ball_touch_section(car, latest_touch)
                    if ball_touch_section is not None:
                        self.cars[true_car_name][friends][foes][zone_id][ball_touch_section] = min(self.cars[true_car_name][friends][foes][zone_id][ball_touch_section] + self.gain, 1)
                        if self.verbose:
                            print(f"{true_car_name} got a {CarHeuristic.NAMES[ball_touch_section]} bonus")
                    elif self.verbose:
                        print(f"{true_car_name} had a shot ignored")

                ball_zone_sections = {}
                for future_ball_zone_id, ball_locations in future_ball_zones.items():
                    if future_ball_zone_id not in ball_zone_sections:
                        ball_zone_sections[future_ball_zone_id] = set()
        
                    for ball_location in tuple(ball_locations):
                        ball_section = self.get_ball_section(ball_location, car.name)

                        if ball_section not in ball_zone_sections[future_ball_zone_id]:
                            ball_zone_sections[future_ball_zone_id].add(ball_section)

                for zone_id, ball_sections in ball_zone_sections.items():
                    for ball_section in ball_sections:
                        self.cars[true_car_name][friends][foes][zone_id][ball_section] = max(self.cars[true_car_name][friends][foes][zone_id][ball_section] - car_loss, 0)
            
        return True

    def get_ball_touch_section(self, car: PlayerInfo, latest_touch: Touch) -> bool:
        time_airborne = self.time - self.car_tracker[car.name]['last_wheel_contact']['time']
        ball_zone = self.get_ball_section(get_hashable_from_vector3(latest_touch.hit_location), latest_touch.player_name)
        if car.has_wheel_contact:
            return 0

        ang_vel_norm = np.linalg.norm(get_np_from_vector3(car.physics.angular_velocity))
        if ball_zone <= 1 and car.jumped and time_airborne < 1.6 and (not car.double_jumped or ang_vel_norm > 4.5):
            return 1

        if ball_zone <= 2 and car.jumped and car.double_jumped and time_airborne < 1.75 and ang_vel_norm <= 5:
            return 2

        if time_airborne > 0.5 or not car.jumped:
            return 3
    
    def get_ball_section(self, ball_location: tuple, car_name: str) -> int:
        location = np.array(ball_location) - self.car_tracker[car_name]['last_wheel_contact']['location']
        
        dbz = abs(self.car_tracker[car_name]['last_wheel_contact']['up'].dot(location))
        divisors = [
            dbz <= 126.75,
            dbz <= 312.75,
            dbz <= 542.75,
            True
        ]

        return divisors.index(True)

    def get_friend_count(self, car_team: int):
        return self.team_count[car_team] - 1
    
    def get_foe_count(self, car_team: int):
        return self.team_count[not car_team]

    def get_zone_id(self, location: Vector3):
        for id_0, zones in enumerate(self.zones):
            for id_1, zone in enumerate(zones):
                if zone != None and zone.intersect_point(location):
                    return id_0 * 3 + id_1

    def get_surrounding_zone_ids(self, zone_id: int) -> List[int]:
        zone_id_0 = zone_id // 3
        zone_id_1 = zone_id % 3
        zone_ids = []

        for id_0 in range(-1, 2, 1):
            id_0 += zone_id_0
            
            if -1 < id_0 < self.field_dimensions[0]:
                for id_1 in range(-1, 2, 1):
                    id_1 += zone_id_1

                    if -1 < id_1 < self.field_dimensions[1] and self.zones[id_0][id_1] is not None:
                        zone_ids.append(id_0 * 3 + id_1)

        return zone_ids

    @staticmethod
    def get_true_car_name(car_name: str) -> str:
        return re.split(r' \(\d+\)$', car_name)[0]

    def get_car(self, car_name: str) -> CarHeuristic:
        true_car_name = self.get_true_car_name(car_name)

        if true_car_name not in self.cars or car_name not in self.car_tracker:
            return None

        prediction_values = deepcopy(self.cars[true_car_name][self.car_tracker[car_name]['friends']][self.car_tracker[car_name]['foes']][self.car_tracker[car_name]['zone_id']])
        zone_ids = self.get_surrounding_zone_ids(self.car_tracker[car_name]['zone_id']) # car tracker zone id is included in this again, but leave it in to give it more weight

        for zone_id in zone_ids:
            zone_prediction_values = self.cars[true_car_name][self.car_tracker[car_name]['friends']][self.car_tracker[car_name]['foes']][zone_id]

            for i in range(len(prediction_values)):
                prediction_values[i] += zone_prediction_values[i]

        num_zones = len(zone_ids) + 1
        for i in range(len(prediction_values)):
            prediction_values[i] /= num_zones

        return prediction_values

    def predict_car(self, car: CarHeuristic) -> dict:
        return {car.NAMES[i]: car[i] > self.threshold for i in range(len(car))}


class Zone2D:
    def __init__(self, min_: np.ndarray, max_: np.ndarray):
        self.min = min_
        self.max = max_

    def intersect_sphere(self, l: np.ndarray, r: float) -> bool:
        nearest = np.array([
            _fcap(l[0], self.min[0], self.max[0]),
            _fcap(l[1], self.min[1], self.max[1]),
            _fcap(l[2], self.min[2], self.max[2])
        ])

        return (l - nearest).magnitude() <= r

    def intersect_point(self, b: Vector3) -> bool:
        return self.min[0] <= b.x and self.max[0] >= b.x and self.min[1] <= b.y and self.max[1] >= b.y


def get_np_from_vector3(vec3: Vector3) -> np.ndarray:
    return np.array([vec3.x, vec3.y, vec3.z])


def get_hashable_from_vector3(vec3: Vector3) -> tuple:
    return (round(vec3.x), round(vec3.y), round(vec3.z))
