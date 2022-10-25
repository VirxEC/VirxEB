from __future__ import annotations

from typing import Tuple

from util import utils
from util.agent import Vector, VirxERLU


def get_cap(agent: VirxERLU, cap_: int, is_anti_shot: bool=False) -> Tuple[float, float] | float:
    if agent.num_friends == 0 or not (agent.me.minimum_time_to_ball > agent.friend_times[0].minimum_time_to_ball and is_anti_shot):
        foes = len(tuple(foe for foe in agent.foes if not foe.demolished and foe.location.y * utils.side(agent.team) < agent.ball.location.y * utils.side(agent.team) - 150))
        if foes != 0 and agent.enemy_time_to_ball != 7:
            future_ball_location_slice = min(round(agent.enemy_time_to_ball * 1.15 * 60), agent.ball_prediction_struct.num_slices - 1)
            foe_factor = max(abs(agent.closest_foes[0].local_velocity().angle2D(agent.closest_foes[0].local_location(agent.ball.location))) * 4, 4)
            foe_intercept_location = agent.ball_prediction_struct.slices[future_ball_location_slice].physics.location
            foe_intercept_location = Vector(foe_intercept_location.x, foe_intercept_location.y)

            cap_slices = round(cap_) * 60
            for i, ball_slice in enumerate(agent.ball_prediction_struct.slices[future_ball_location_slice:cap_slices:2]):
                ball_loc = Vector(ball_slice.physics.location.x, ball_slice.physics.location.y)

                if foe_intercept_location.dist(ball_loc) >= agent.ball_radius * foe_factor + (agent.me.hitbox.width * (2 + foes)):
                    cap_ = (i - 1) / 60
                    break

    return cap_
