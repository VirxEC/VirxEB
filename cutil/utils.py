from __future__ import annotations

from queue import Full
from typing import Tuple

import virx_erlu_rlib as rlru
from util import utils
from util.agent import Vector, VirxERLU


def get_cap(agent: VirxERLU, cap_: int, is_anti_shot: bool=False) -> Tuple[float, float] | float:
    if agent.num_friends == 0 or not (agent.me.minimum_time_to_ball > agent.friend_times[0].minimum_time_to_ball and is_anti_shot):
        foes = len(tuple(foe for foe in agent.foes if not foe.demolished and foe.location.y * utils.side(agent.team) < agent.ball.location.y * utils.side(agent.team) - 150))
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
