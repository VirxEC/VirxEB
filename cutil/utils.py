from __future__ import annotations

from queue import Full
from typing import Tuple

import numpy as np
import virx_erlu_rlib as rlru
from numba import njit

from util import utils
from util.agent import Vector, VirxERLU


@njit('Array(float32, 1, "C")(Array(float32, 2, "C"), Array(float32, 1, "C"), Array(float32, 1, "C"), int8)', fastmath=True, cache=True)
def get_clamp_target(friend_goal: np.ndarray, ball: np.ndarray, closest_foe_location: np.ndarray, self_team: int) -> np.ndarray:
    real_ball = ball * np.array((0, self_team, 0), dtype=np.float32)
    s = real_ball - closest_foe_location
    s /= np.linalg.norm(s)
    start = friend_goal[1] - real_ball
    start /= np.linalg.norm(start)
    end = friend_goal[2] - real_ball
    end /= np.linalg.norm(end)

    neg_one = np.array((0, 0, -1), dtype=np.float32)
    right = np.dot(s, np.cross(end, neg_one)) < 0
    left = np.dot(s, np.cross(start, neg_one)) > 0

    if (right and left) if np.dot(end, np.cross(start, neg_one)) > 0 else (right or left):
        target = np.array((0, friend_goal[0][1], 0), dtype=np.float32)
        p1 = real_ball
        p2 = real_ball - s
        x_diff = p2[0] - p1[0]

        if x_diff == 0:
            target[0] = 0
        else:
            m = (p2[1] - p1[1]) / (p2[0] - p1[0])
            b = p1[1] - (m * p1[0])
            # x = (y - b) / m
            target[0] = (target[1] - b) / m

        return target
    elif np.dot(start, s) < np.dot(end, s):
        return friend_goal[2].copy()
    else:
        return friend_goal[1].copy()


@njit('Array(float32, 1, "C")(Array(float32, 2, "C"), Array(float32, 2, "C"), Array(float32, 1, "C"), int8)', fastmath=True, cache=True)
def get_traditional_target(friends: np.ndarray, friend_goal: np.ndarray, ball: np.ndarray, self_team: int) -> np.ndarray:
    horizontal_offset = 400

    if ball[1] * self_team < -640:
        target = friend_goal[0].copy()
        target_x = target[0]

        step = 2
        while utils.friend_near_target(friends, np.array((target_x, target[1]), dtype=np.float32)):
            offset = horizontal_offset * step / 2
            if step % 2 == 1:
                offset *= -1
            target_x = target[0] + offset
            step += 1
        target[0] = target_x

        return target

    if ball[0] * self_team < 0:
        close_post = friend_goal[1].copy()
        far_post = friend_goal[2].copy()
    else:
        far_post = friend_goal[1].copy()
        close_post = friend_goal[2].copy()

    mid_post = friend_goal[0].copy()
    mid_post[0] = far_post[0] / 2
    side = utils._fsign(close_post[0])

    if not utils.friend_near_target(friends, close_post):
        return close_post

    if not utils.friend_near_target(friends, mid_post):
        return mid_post

    target = far_post
    while utils.friend_near_target(friends, target):
        target[0] += horizontal_offset * side

    return target


def get_cap(agent: VirxERLU, cap_: int, is_anti_shot: bool=False) -> Tuple[float, float] | float:
    if agent.num_friends == 0 or not (agent.me.minimum_time_to_ball > agent.friend_times[0].minimum_time_to_ball and is_anti_shot):
        foes = len(tuple(foe for foe in agent.foes if not foe.demolished and foe.location.y * agent.friend_team_side < agent.ball.location.y * agent.friend_team_side - 150))
        if foes != 0 and agent.enemy_time_to_ball != 7:
            future_ball_location_slice = min(round(agent.enemy_time_to_ball * 1.15 * 120), rlru.get_num_ball_slices() - 1)
            foe_factor = max(abs(agent.closest_foes[0].local_velocity().angle2D(agent.closest_foes[0].local_location(agent.ball.location))) * 4, 4)
            foe_intercept_location = rlru.get_slice_index(future_ball_location_slice).location
            foe_intercept_location = Vector(foe_intercept_location[0], foe_intercept_location[1])

            cap_slices = round(cap_) * 120
            for i in range(future_ball_location_slice, cap_slices, 4):
                ball_slice = rlru.get_slice_index(i)
                ball_loc = Vector(ball_slice.location[0], ball_slice.location[0])

                if foe_intercept_location.dist(ball_loc) >= agent.ball_radius * foe_factor + (agent.me.hitbox.width * (2 + foes)):
                    cap_ = (i - 1) / 120
                    break

    return cap_


def send_comm(agent: VirxERLU, msg: dict):
    message = {
        "index": agent.index,
        "team": agent.team
    }
    msg.update(message)
    try:
        agent.matchcomms.outgoing_broadcast.put_nowait({
            "VirxERLU": msg
        })
    except Full:
        agent.print("Outgoing broadcast is full; couldn't send message")
