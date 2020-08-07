import virxrlcu
from util.routines import Aerial, jump_shot
from util.utils import Vector, math


def find_jump_shot(agent, target, cap_=6):
    # Takes a tuple of (left,right) target pairs and finds routines that could hit the ball between those target pairs
    # Only meant for routines that require a defined intercept time/place in the future
    # This can take a lot of time so it uses a C function to do a lot of the math, speeding the process up
    slices = get_slices(agent, cap_)

    if slices is None:
        return

    target = (
        target[0].tuple(),
        target[1].tuple()
    )

    me = (
        agent.me.location.tuple(),
        agent.me.forward.tuple(),
        agent.me.boost
    )

    # Look at every 3rd slice, starting at 0.2 seconds into the future
    for ball_slice in slices:
        # Gather some data about the slice
        intercept_time = ball_slice.game_seconds
        time_remaining = intercept_time - agent.time

        if time_remaining <= 0:
            continue

        ball_location = (ball_slice.physics.location.x, ball_slice.physics.location.y, ball_slice.physics.location.z)

        if abs(ball_location[1]) > 5212:
            return  # abandon search if ball is scored at/after this point

        # If the ball is above what this function can handle, don't bother with any further processing and skip to the next slice
        if ball_location[2] > 350:
            continue

        # Check if we can make a shot at this slice
        # This operation is very expensive, so we use a custom C function to improve run time
        shot = virxrlcu.parse_slice_for_jump_shot_with_target(time_remaining, ball_location, *me, *target, cap_)

        # If we found a viable shot, pass the data into the shot routine and return the shot
        if shot['found'] == 1:
            return jump_shot(Vector(*ball_location), intercept_time, Vector(*shot['best_shot_vector']), shot['direction'])


def find_any_jump_shot(agent, cap_=3):
    slices = get_slices(agent, cap_)

    if slices is None:
        return

    me = (
        agent.me.location.tuple(),
        agent.me.forward.tuple(),
        agent.me.boost
    )

    for ball_slice in slices:
        intercept_time = ball_slice.game_seconds
        time_remaining = intercept_time - agent.time

        if time_remaining <= 0:
            continue

        ball_location = (ball_slice.physics.location.x, ball_slice.physics.location.y, ball_slice.physics.location.z)

        if abs(ball_location[1]) > 5212:
            return

        if ball_location[2] > 350:
            continue

        shot = virxrlcu.parse_slice_for_jump_shot(time_remaining, ball_location, *me, cap_)

        if shot['found'] == 1:
            return jump_shot(Vector(*ball_location), intercept_time, Vector(*shot['best_shot_vector']), shot['direction'])


def find_aerial(agent, target, cap_=4):
    slices = get_slices(agent, cap_)

    if slices is None:
        return

    max_aerial_height = math.inf

    target = (
        target[0].tuple(),
        target[1].tuple()
    )

    me = (
        agent.me.location.tuple(),
        agent.me.velocity.tuple(),
        agent.me.up.tuple(),
        agent.me.forward.tuple(),
        1 if agent.me.airborne else -1,
        agent.me.boost
    )

    gravity = agent.gravity.tuple()

    if len(agent.friends) == 0 and len(agent.foes) == 1:
        max_aerial_height = 643

    for ball_slice in slices:
        intercept_time = ball_slice.game_seconds
        time_remaining = intercept_time - agent.time

        if time_remaining <= 0:
            return

        ball_location = (ball_slice.physics.location.x, ball_slice.physics.location.y, ball_slice.physics.location.z)

        if abs(ball_location[2]) > 5212:
            return

        if 275 > ball_location[2] or ball_location[2] > max_aerial_height:
            continue

        shot = virxrlcu.parse_slice_for_aerial_shot_with_target(time_remaining, agent.boost_accel, gravity, ball_location, me, *target, cap_)

        if shot['found'] == 1:
            return Aerial(Vector(*shot['ball_intercept']), intercept_time)


def find_any_aerial(agent, cap_=3):
    slices = get_slices(agent, cap_)

    if slices is None:
        return

    max_aerial_height = math.inf

    me = (
        agent.me.location.tuple(),
        agent.me.velocity.tuple(),
        agent.me.up.tuple(),
        agent.me.forward.tuple(),
        1 if agent.me.airborne else -1,
        agent.me.boost
    )

    gravity = agent.gravity.tuple()

    if len(agent.friends) == 0 and len(agent.foes) == 1:
        max_aerial_height = 643

    for ball_slice in slices:
        intercept_time = ball_slice.game_seconds
        time_remaining = intercept_time - agent.time

        if time_remaining <= 0:
            return

        ball_location = (ball_slice.physics.location.x, ball_slice.physics.location.y, ball_slice.physics.location.z)

        if abs(ball_location[1]) > 5212:
            return

        if 275 > ball_location[2] or ball_location[2] > max_aerial_height:
            continue

        shot = virxrlcu.parse_slice_for_aerial_shot(time_remaining, agent.boost_accel, gravity, ball_location, me, cap_)

        if shot['found'] == 1:
            return Aerial(Vector(*shot['ball_intercept']), intercept_time)


def get_slices(agent, cap_):
    # Get the struct
    struct = agent.predictions['ball_struct']

    # Make sure it isn't empty
    if struct is None:
        return

    # If we're shooting, crop the struct
    if agent.shooting:
        time_remaining = agent.shot_time - agent.time

        if time_remaining <= 0.2:
            return

        end_slice = math.ceil(time_remaining * 60)

        # Half the time, double the slices
        if time_remaining <= 3:
            return struct.slices[12:end_slice]

        return struct.slices[12:end_slice:2]

    # Cap the slices... at the cap...
    end_slice = math.ceil(cap_ * 60)

    # Start 0.2 seconds in, and skip every other data point
    return struct.slices[12:end_slice:2]
