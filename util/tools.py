from enum import Enum

from util.routines import Aerial, double_jump, ground_shot, jump_shot, virxrlcu
from util.utils import Vector, cap, math, side, get_cap


class ShotType(Enum):
    GROUND = 0
    JUMP = 1
    DOUBLE_JUMP = 2
    AERIAL = 3


SHOT_SWITCH = {
    ShotType.GROUND: ground_shot,
    ShotType.JUMP: jump_shot,
    ShotType.DOUBLE_JUMP: double_jump
}


def find_ground_shot(agent, target, weight=None, cap_=6):
    return find_shot(agent, target, weight, cap_, can_aerial=False, can_double_jump=False, can_jump=False)


def find_any_ground_shot(agent, cap_=6):
    return find_any_shot(agent, cap_, can_aerial=False, can_double_jump=False, can_jump=False)


def find_jump_shot(agent, target, weight=None, cap_=6):
    return find_shot(agent, target, weight, cap_, can_aerial=False, can_double_jump=False, can_ground=False)


def find_any_jump_shot(agent, cap_=6):
    return find_any_shot(agent, cap_, can_aerial=False, can_double_jump=False, can_ground=False)


def find_double_jump(agent, target, weight=None, cap_=6):
    return find_shot(agent, target, weight, cap_, can_aerial=False, can_jump=False, can_ground=False)


def find_any_double_jump(agent, cap_=6):
    return find_any_shot(agent, cap_, can_aerial=False, can_jump=False, can_ground=False)


def find_aerial(agent, target, weight=None, cap_=6):
    return find_shot(agent, target, weight, cap_, can_double_jump=False, can_jump=False, can_ground=False)


def find_any_aerial(agent, cap_=6):
    return find_any_shot(agent, cap_, can_double_jump=False, can_jump=False, can_ground=False)


def find_shot(agent, target, weight=None, cap_=6, can_aerial=True, can_double_jump=True, can_jump=True, can_ground=True):
    if not can_aerial and not can_double_jump and not can_jump and not can_ground:
        agent.print("WARNING: All shots were disabled when find_shot was ran")
        return

    # Takes a tuple of (left,right) target pair or (right,left) anti-target pair and finds routines that could hit the ball between those target pairs
    # Only meant for routines that require a defined intercept time/place in the future

    cap_, aerial_time_cap = get_cap(agent, cap_, True)

    # Assemble data in a form that can be passed to C
    targets = (
        tuple(target[0]),
        tuple(target[1])
    )

    me = agent.me.get_raw(agent)

    game_info = (
        agent.boost_accel,
        agent.ball_radius
    )

    gravity = tuple(agent.gravity)

    max_aerial_height = 1000 if len(agent.friends) == 0 and len(agent.foes) == 1 else math.inf
    min_aerial_height = 551 if max_aerial_height > 1000 and agent.me.location.z >= 2044 - agent.me.hitbox.height * 1.1 else 300
    self_team = side(agent.team)
    self_loc = agent.me.location.y * self_team

    is_on_ground = not agent.me.airborne
    can_ground = is_on_ground and can_ground
    can_jump = is_on_ground and can_jump
    can_double_jump = is_on_ground and can_double_jump

    if not can_ground and not can_jump and not can_double_jump and not can_aerial:
        return

    # Here we get the slices that need to be searched - by defining a cap, we can reduce the number of slices and improve search times
    slices = get_slices(agent, cap_, weight=weight)

    if slices is None:
        return

    # Loop through the slices
    for ball_slice in slices:
        # Gather some data about the slice
        intercept_time = ball_slice.game_seconds
        T = intercept_time - agent.time - (1 / 120)

        if T <= 0:
            return

        ball_location = (ball_slice.physics.location.x, ball_slice.physics.location.y, ball_slice.physics.location.z)

        if abs(ball_location[1]) > 5212.75:
            return  # abandon search if ball is scored at/after this point

        ball_info = (ball_location, (ball_slice.physics.velocity.x, ball_slice.physics.velocity.y, ball_slice.physics.velocity.z))

        # Check if we can make a shot at this slice
        # This operation is very expensive, so we use C to improve run time
        shot = virxrlcu.parse_slice_for_shot_with_target(can_ground, can_jump, can_double_jump, can_aerial and (min_aerial_height < ball_location[2] < max_aerial_height) and T < aerial_time_cap and ball_location[1] * self_team < self_loc, T, *game_info, gravity, ball_info, me, targets)

        if shot['found'] == 1:
            shot_type = ShotType(shot["shot_type"])
            if shot_type == ShotType.AERIAL:
                return Aerial(intercept_time, (Vector(*shot['targets'][0]), Vector(*shot['targets'][1])), shot['fast'])

            return SHOT_SWITCH[shot_type](intercept_time, (Vector(*shot['targets'][0]), Vector(*shot['targets'][1])))


def find_any_shot(agent, cap_=6, can_aerial=True, can_double_jump=True, can_jump=True, can_ground=True):
    if not can_aerial and not can_double_jump and not can_jump and not can_ground:
        agent.print("WARNING: All shots were disabled when find_any_shot was ran")
        return

    # Only meant for routines that require a defined intercept time/place in the future

    cap_, aerial_time_cap = get_cap(agent, cap_, True)

    # Assemble data in a form that can be passed to C
    me = agent.me.get_raw(agent)

    game_info = (
        agent.boost_accel,
        agent.ball_radius
    )

    gravity = tuple(agent.gravity)

    max_aerial_height = 1000 if len(agent.friends) == 0 and len(agent.foes) == 1 else math.inf
    min_aerial_height = 551 if max_aerial_height > 1000 and agent.me.location.z >= 2044 - agent.me.hitbox.height * 1.1 else 300

    is_on_ground = not agent.me.airborne
    can_ground = is_on_ground and can_ground
    can_jump = is_on_ground and can_jump
    can_double_jump = is_on_ground and can_double_jump

    if not can_ground and not can_jump and not can_double_jump and not can_aerial:
        return

    # Here we get the slices that need to be searched - by defining a cap, we can reduce the number of slices and improve search times
    slices = get_slices(agent, cap_)

    if slices is None:
        return

    # Loop through the slices
    for ball_slice in slices:
        # Gather some data about the slice
        intercept_time = ball_slice.game_seconds
        T = intercept_time - agent.time - (1 / 120)

        if T <= 0:
            return

        ball_location = (ball_slice.physics.location.x, ball_slice.physics.location.y, ball_slice.physics.location.z)

        if abs(ball_location[1]) > 5212.75:
            return  # abandon search if ball is scored at/after this point

        ball_info = (ball_location, (ball_slice.physics.velocity.x, ball_slice.physics.velocity.y, ball_slice.physics.velocity.z))

        # Check if we can make a shot at this slice
        # This operation is very expensive, so we use C to improve run time
        shot = virxrlcu.parse_slice_for_shot(can_ground, can_jump, can_double_jump, can_aerial and (min_aerial_height < ball_location[2] < max_aerial_height) and T < aerial_time_cap, T, *game_info, gravity, ball_info, me)

        if shot['found'] == 1:
            shot_type = ShotType(shot["shot_type"])
            if shot_type == ShotType.AERIAL:
                return Aerial(intercept_time, fast_aerial=shot['fast'])

            return SHOT_SWITCH[shot_type](intercept_time)


def get_slices(agent, cap_, weight=None, start_slice=6):
    # Get the struct
    struct = agent.ball_prediction_struct
    min_time_to_ball = max(agent.predictions['self_min_time_to_ball'] / 3 * 2, 0)

    # Make sure it isn't empty
    if struct is None or min_time_to_ball > cap_:
        return

    if start_slice / 60 < min_time_to_ball:
        start_slice = round(min_time_to_ball * 60) - 1

    end_slices = None

    # If we're shooting, crop the struct
    if agent.shooting and agent.shot_weight != -1:
        # Get the time remaining
        time_remaining = agent.stack[0].intercept_time - agent.time
        if time_remaining < 0.5 and time_remaining >= 0:
            return

        # if the shot is done but it's working on it's 'follow through', then ignore this stuff
        if time_remaining > 0:
            # Convert the time remaining into number of slices, and take off the minimum gain accepted from the time
            min_gain = 0.05 if weight is None or weight is agent.shot_weight else -(agent.max_shot_weight - agent.shot_weight + 1)
            end_slice = round(min(time_remaining - min_gain, cap_) * 60)

    if end_slices is None:
        # Cap the slices
        end_slice = round(cap_ * 60)

    # We can't end a slice index that's lower than the start index
    if end_slice <= start_slice:
        return

    # for every second worth of slices that we have to search, skip 1 more slice (for performance reasons) - min 1 and max 3
    skip = cap(end_slice - start_slice / 60, 1, 3)
    return struct.slices[start_slice:end_slice:skip]
