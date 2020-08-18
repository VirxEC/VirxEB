from util.utils import (Vector, almost_equals, backsolve, cap, defaultPD,
                        defaultThrottle, math, peek_generator, shot_valid,
                        side, sign, valid_ceiling_shot)

max_speed = 2300
throttle_accel = 100 * (2/3)
brake_accel = Vector(x=-3500)
boost_per_second = 33 + (1/3)
jump_max_duration = 0.2
jump_speed = 291 + (2/3)
jump_acc = 1458 + (1/3)


class atba:
    def __init__(self, exit_distance=500, exit_flip=True):
        self.exit_distance = exit_distance
        self.exit_flip = exit_flip

    def run(self, agent):
        target = agent.ball.location
        car_to_target = target - agent.me.location
        distance_remaining = car_to_target.flatten().magnitude()

        # Some adjustment to the final target to ensure it's inside the field and we don't try to drive through any goalposts to reach it
        if abs(agent.me.location.y) > 5150:
            target.x = cap(target.x, -750, 750)

        local_target = agent.me.local(target - agent.me.location)

        angles = defaultPD(agent, local_target)
        defaultThrottle(agent, 1400)

        agent.controller.boost = False
        agent.controller.handbrake = True if abs(angles[1]) >= 2.3 or (agent.me.local(agent.me.velocity).x >= 900 and abs(angles[1]) > 1.57) else agent.controller.handbrake

        velocity = 1+agent.me.velocity.magnitude()
        if distance_remaining < self.exit_distance:
            agent.pop()
            if self.exit_flip:
                agent.push(flip(local_target))
        elif abs(angles[1]) < 0.05 and velocity > 600 and velocity < 2150 and distance_remaining / velocity > 2.0:
            agent.push(flip(local_target))
        elif abs(angles[1]) > 2.8 and velocity < 200:
            agent.push(flip(local_target, True))
        elif agent.me.airborne:
            agent.push(recovery(target))


class wave_dash:
    def __init__(self):
        self.step = 0

    def run(self, agent):
        if self.step <= 5:
            self.step += 1
            agent.controller.pitch = 1
            agent.controller.yaw = agent.controller.role = 0

        if self.step <= 1:
            agent.controller.jump = True
        elif self.step <= 4:
            agent.controller.jump = False
        else:
            if (agent.me.location + (agent.me.velocity * 0.2)).z < 5:
                agent.controller.jump = True
                agent.controller.pitch = -1
                agent.controller.yaw = agent.controller.role = 0
                agent.pop()
            elif not agent.me.airborne:
                agent.pop()


class Aerial:
    def __init__(self, ball_intercept, intercept_time):
        self.target = ball_intercept
        self.intercept_time = intercept_time
        self.jumping = True
        self.ceiling = False
        self.time = -1
        self.jump_time = -1
        self.counter = 0

    def run(self, agent):
        if not agent.shooting:
            agent.shooting = True

        if self.time is -1:
            elapsed = 0
            self.time = agent.time
            agent.print(f"Hit ball via aerial {round(agent.me.location.dist(self.target), 4)}uu's away in {round(self.intercept_time - self.time, 4)}s")
        else:
            elapsed = agent.time - self.time

        T = self.intercept_time - agent.time
        xf = agent.me.location + agent.me.velocity * T + 0.5 * agent.gravity * T ** 2
        vf = agent.me.velocity + agent.gravity * T

        if self.jumping and agent.me.location.z < 2044 - agent.me.hitbox.height * 1.1:
            agent.dbg_2d(f"Jumping")
            if self.jump_time == -1:
                jump_elapsed = 0
                self.jump_time = agent.time
            else:
                jump_elapsed = agent.time - self.jump_time

            tau = jump_max_duration - jump_elapsed

            if jump_elapsed == 0:
                vf += agent.me.up * jump_speed
                xf += agent.me.up * jump_speed * T

            vf += agent.me.up * jump_acc * tau
            xf += agent.me.up * jump_acc * tau * (T - 0.5 * tau)

            vf += agent.me.up * jump_speed
            xf += agent.me.up * jump_speed * (T - tau)

            if jump_elapsed < jump_max_duration:
                agent.controller.jump = True
            elif jump_elapsed >= jump_max_duration and self.counter < 3:
                self.counter += 1
            elif jump_elapsed < 0.3:
                agent.controller.jump = True
            else:
                self.jumping = jump_elapsed <= 0.3
        elif self.jumping:
            self.jumping = False
            self.ceiling = True
            self.target -= Vector(z=92)

        if self.ceiling:
            agent.dbg_2d(f"Ceiling shot")

        delta_x = self.target - xf
        direction = delta_x.normalize()

        agent.line(agent.me.location, self.target, agent.renderer.white())
        agent.line(self.target - Vector(z=100), self.target + Vector(z=100), agent.renderer.green())
        agent.line(agent.me.location, direction, agent.renderer.red())

        defaultPD(agent, agent.me.local(delta_x if delta_x.magnitude() > 50 else self.target), upside_down=self.ceiling)

        if jump_max_duration <= elapsed and elapsed < 0.3 and self.counter ** 3:
            agent.controller.roll = agent.controller.pitch = agent.controller.yaw = agent.controller.steer = 0

        if agent.me.forward.angle3D(direction) < 0.3:
            if delta_x.magnitude() > 50:
                agent.controller.boost = 1
            else:
                agent.controller.throttle = cap(0.5 * throttle_accel * T * T, 0, 1)
        else:
            agent.controller.boost = agent.controller.throttle = 0

        still_valid = shot_valid(agent, self, threshold=250, target=self.target)

        if T <= 0 or not still_valid:
            if not still_valid:
                agent.print("Aerial is no longer valid")

            agent.pop()
            agent.shooting = False
            agent.shot_weight = -1
            agent.shot_time = -1
            agent.push(ball_recovery())
        elif self.ceiling and self.target.dist(agent.me.location) < 92 + agent.me.hitbox.length and not agent.me.doublejumped and agent.me.location.z < agent.ball.location.z + 92 and self.target.y * side(agent.team) > -4240:
            agent.dbg_2d("Flipping")
            agent.controller.jump = True
            local_target = agent.me.local(self.target - agent.me.location)
            agent.controller.pitch = abs(local_target.x) * -sign(local_target.x)
            agent.controller.yaw = abs(local_target.y) * sign(local_target.y)


class flip:
    # Flip takes a vector in local coordinates and flips/dodges in that direction
    # cancel causes the flip to cancel halfway through, which can be used to half-flip
    def __init__(self, vector, cancel=False):
        self.vector = vector.normalize()
        self.pitch = abs(self.vector.x) * -sign(self.vector.x)
        self.yaw = abs(self.vector.y) * sign(self.vector.y)
        self.cancel = cancel
        # the time the jump began
        self.time = -1
        # keeps track of the frames the jump button has been released
        self.counter = 0

    def run(self, agent, manual=False):
        if agent.gravity.z >= 3250:
            agent.pop()

        if self.time == -1:
            elapsed = 0
            self.time = agent.time
        else:
            elapsed = agent.time - self.time

        if elapsed < 0.15:
            agent.controller.jump = True
        elif elapsed >= 0.15 and self.counter < 3:
            agent.controller.jump = False
            self.counter += 1
        elif elapsed < 0.3 or (not self.cancel and elapsed < 0.9):
            agent.controller.jump = True
            agent.controller.pitch = self.pitch
            agent.controller.yaw = self.yaw
        elif manual:
            return True
        else:
            agent.pop()
            agent.push(recovery())


class brake:
    def run(self, agent, manual=False):
        speed = agent.me.local(agent.me.velocity).x
        if speed > 0:
            agent.controller.throttle = -1
            if speed < 25 and not manual:
                agent.pop()
        elif speed < 0:
            agent.controller.throttle = 1
            if speed > -25 and not manual:
                agent.pop()
        elif not manual:
            agent.pop()


class goto:
    # Drives towards a designated (stationary) target
    # Optional vector controls where the car should be pointing upon reaching the target
    def __init__(self, target, vector=None, brake=False):
        self.target = target
        self.vector = vector
        self.brake = brake

    def run(self, agent, manual=False):
        car_to_target = self.target - agent.me.location
        distance_remaining = car_to_target.flatten().magnitude()
        angle_to_target = abs(Vector(x=1).angle(agent.me.local(car_to_target)))
        direction = 1 if angle_to_target < 2.2 else -1

        agent.dbg_2d(f"Angle to target: {angle_to_target}")
        agent.dbg_2d(f"Distance to target: {distance_remaining}")
        agent.line(self.target - Vector(z=500), self.target + Vector(z=500), [255, 0, 255])

        if (not self.brake and distance_remaining < 350) or (self.brake and distance_remaining < (agent.me.local(agent.me.velocity).x ** 2 * -1) / (2 * brake_accel.x)):
            if not manual:
                agent.pop()

            if self.brake:
                agent.push(brake())
            return

        if self.vector is not None:
            # See comments for adjustment in jump_shot for explanation
            side_of_vector = sign(self.vector.cross(Vector(z=1)).dot(car_to_target))
            car_to_target_perp = car_to_target.cross(Vector(z=side_of_vector)).normalize()
            adjustment = car_to_target.angle(self.vector) * distance_remaining / 3.14
            final_target = self.target + (car_to_target_perp * adjustment)
        else:
            final_target = self.target

        # Some adjustment to the final target to ensure it's inside the field and we don't try to drive through any goalposts to reach it
        if abs(agent.me.location.y) > 5120 + (agent.me.hitbox.length / 2):
            final_target.x = cap(final_target.x, -750, 750)

        local_target = agent.me.local(final_target - agent.me.location)

        defaultPD(agent, local_target, direction)
        target_speed = 2300 if distance_remaining > 1280 else 1400
        defaultThrottle(agent, target_speed, direction)

        if len(agent.friends) > 0 and agent.me.local(agent.me.velocity).x < 10 and agent.controller.throttle == 1 and min(agent.me.location.flat_dist(car.location) for car in agent.friends) < 251:
            agent.push(flip(Vector(y=250)))
            return

        if agent.me.boost < 30 or (agent.playstyle is agent.playstyles.Defensive and agent.predictions['self_from_goal'] < 4000):
            agent.controller.boost = False
        agent.controller.handbrake = (angle_to_target >= 2.3 or (agent.me.local(agent.me.velocity).x >= 900 and angle_to_target > 1.57)) and direction == 1

        velocity = 1+agent.me.velocity.magnitude()
        if angle_to_target < 0.05 and distance_remaining > 1920 and velocity > 600 and velocity < 2150 and distance_remaining / velocity > 2:
            agent.push(flip(local_target))
        elif angle_to_target >= 1.57 and distance_remaining > 1000 and velocity < 200 and distance_remaining / velocity > 2:
            agent.push(flip(local_target, True))
        elif agent.me.airborne:
            agent.push(recovery(self.target))


class dynamic_backcheck:
    def __init__(self):
        self.start_time = None
        self.goto = goto(Vector(), brake=True)

    def run(self, agent):
        if self.start_time is None:
            self.start_time = agent.time

        ball_slice = agent.predictions['ball_struct'].slices[agent.future_ball_location_slice].physics.location
        ball_loc = Vector(ball_slice.x, ball_slice.y)
        agent.line(ball_loc, ball_loc + Vector(z=185), agent.renderer.white())
        ball_loc.y *= side(agent.team)

        if agent.time - self.start_time > 0.5 or agent.playstyle is agent.playstyles.Defensive or ball_loc.y > 2560:
            agent.pop()
            return

        distance = 1280 if len(agent.friends) >= 2 or agent.playstyle is agent.playstyles.Offensive else 2560

        target = Vector(y=(ball_loc.y + distance) * side(agent.team))
        agent.line(target, target + Vector(z=642), (255, 0, 255))

        if ball_loc.x > 2560:
            target.x = 2560 if ball_loc.y <= 0 else 1024
        elif ball_loc.x < -2560:
            target.x = -2560 if ball_loc.y <= 0 else -1024

        self_to_target = agent.me.location.dist(target)

        if self_to_target > 350:
            self.goto.target = target
            ball_loc.y *= side(agent.team)
            self.goto.vector = ball_loc
            self.goto.run(agent, manual=True)

            if self_to_target < 500:
                agent.controller.boost = False
                agent.controller.throttle = cap(agent.controller.throttle, -0.75, 0.75)


class retreat:
    def __init__(self):
        self.goto = goto(Vector())
        self.brake = brake()

    def run(self, agent):
        ball_slice = agent.predictions['ball_struct'].slices[agent.future_ball_location_slice].physics.location
        ball = Vector(ball_slice.x, ball_slice.y)
        agent.line(ball, ball + Vector(z=185), agent.renderer.white())

        if ball.y * side(agent.team) < 2560 and agent.playstyle is not agent.playstyles.Defensive:
            agent.pop()
            agent.push(dynamic_backcheck())
            return

        target = self.get_target(agent)
        agent.line(target, target + Vector(z=642), (255, 0, 255))

        if target.flat_dist(agent.me.location) < 350:
            if agent.me.velocity.magnitude() > 100:
                self.brake.run(agent)
            elif abs(Vector(x=1).angle(agent.me.local(ball - agent.me.location))) > 0.25:
                agent.push(face_target(ball=True))
            else:
                agent.pop()
        else:
            self.goto.target = target
            self.goto.run(agent, manual=True)

    def get_target(self, agent):
        target = None
        ball_slice = agent.predictions['ball_struct'].slices[agent.future_ball_location_slice].physics.location
        ball = Vector(ball_slice.x, ball_slice.y, ball_slice.z)
        ball_y = ball.y * side(agent.team)

        team_to_ball = [car.location.flat_dist(ball) for car in agent.friends if car.location.y * side(agent.team) >= ball_y - 50 and abs(car.location.x) < abs(ball.x)]
        self_to_ball = agent.me.location.flat_dist(ball)
        team_to_ball.append(self_to_ball)
        team_to_ball.sort()

        if agent.me.location.y * side(agent.team) >= ball_y - 50 and abs(agent.me.location.x) < abs(ball.x):
            if len(agent.friends) == 0 or abs(ball.x) < 900 or team_to_ball[-1] is self_to_ball:
                target = agent.friend_goal.location
            elif team_to_ball[0] is self_to_ball:
                target = agent.friend_goal.right_post if abs(ball.x) > 10 else agent.friend_goal.left_post

        if target is None:
            if len(agent.friends) <= 1:
                target = agent.friend_goal.location
            else:
                target = agent.friend_goal.left_post if abs(ball.x) > 10 else agent.friend_goal.right_post

        target = target.copy()
        target.y += 250 * side(agent.team) if len(agent.friends) == 0 or abs(ball.x) < 900 or team_to_ball[-1] is self_to_ball else -245 * side(agent.team)

        return target.flatten()


class face_target:
    def __init__(self, target=None, ball=False):
        self.target = target
        self.ball = ball
        self.counter = 0

    def run(self, agent):
        if self.counter == 0:
            agent.controller.jump = True
            self.counter += 1
        elif self.counter == 1:
            agent.pop()
            if self.ball:
                agent.push(ball_recovery())
            else:
                agent.push(recovery(self.target))


class goto_boost:
    # very similar to goto() but designed for grabbing boost
    def __init__(self, boost):
        self.boost = boost
        self.start_time = None
        self.goto = goto(self.boost.location)

    def run(self, agent):
        if self.start_time is None:
            self.start_time = agent.time
            if not self.boost.large:
                self.goto.vector = agent.ball.location
        elif agent.time - self.start_time > 2:
            agent.pop()
        else:
            self.goto.run(agent)


class jump_shot:
    # Hits a target point at a target time towards a target direction
    # Target must be no higher than 300uu unless you're feeling lucky
    def __init__(self, ball_location, intercept_time, shot_vector, best_shot_value):
        self.ball_location = ball_location
        self.intercept_time = intercept_time
        # The direction we intend to hit the ball in
        self.shot_vector = shot_vector
        # The point we dodge at
        self.dodge_point = self.ball_location - (self.shot_vector * best_shot_value)
        # Flags for what part of the routine we are in
        self.jumping = False
        self.dodging = False
        self.counter = 0

    def run(self, agent):
        agent.shooting = True
        raw_time_remaining = self.intercept_time - agent.time
        # Capping raw_time_remaining above 0 to prevent division problems
        time_remaining = cap(raw_time_remaining, 0.001, 10.0)
        car_to_ball = self.ball_location - agent.me.location
        # whether we should go forwards or backwards
        angle_to_target = abs(Vector(x=1).angle(agent.me.local(car_to_ball)))
        direction = 1 if angle_to_target < 2.2 else -1
        # whether we are to the left or right of the shot vector
        side_of_shot = sign(self.shot_vector.cross(Vector(z=1)).dot(car_to_ball))

        car_to_dodge_point = self.dodge_point - agent.me.location
        car_to_dodge_perp = car_to_dodge_point.cross(Vector(z=side_of_shot))  # perpendicular
        distance_remaining = car_to_dodge_point.magnitude()

        speed_required = distance_remaining / time_remaining
        acceleration_required = backsolve(self.dodge_point, agent.me, time_remaining, 0 if not self.jumping else agent.gravity.z)
        local_acceleration_required = agent.me.local(acceleration_required)

        # The adjustment causes the car to circle around the dodge point in an effort to line up with the shot vector
        # The adjustment slowly decreases to 0 as the bot nears the time to jump
        adjustment = car_to_dodge_point.angle(self.shot_vector) * distance_remaining / 2.0  # size of adjustment
        # controls how soon car will jump based on acceleration required
        # If we're angled off really far from the dodge point, then we'll delay the jump in an effort to get a more accurate shot
        # If we're dead on with the shot (ex 0.25 radians off) then we'll jump a lot sooner
        # any number larger than 0 works for the minimum
        # 584 is the highest you can go, for the maximum
        jump_threshold = cap(abs(Vector(x=1).angle(agent.me.local(self.dodge_point - agent.me.location))) * 400, 100, 500)
        # factoring in how close to jump we are
        adjustment *= (cap(jump_threshold - (acceleration_required.z), 0, jump_threshold) / jump_threshold)
        # we don't adjust the final target if we are already jumping
        final_target = self.dodge_point + ((car_to_dodge_perp.normalize() * adjustment) if not self.jumping else 0) + Vector(z=50)
        # Ensuring our target isn't too close to the sides of the field, where our car would get messed up by the radius of the curves

        # Some adjustment to the final target to ensure it's inside the field and we don't try to drive through any goalposts to reach it
        if abs(agent.me.location.y) > 5120 - (agent.me.hitbox.width / 2):
            final_target.x = cap(final_target.x, -750, 750)

        local_final_target = agent.me.local(final_target - agent.me.location)

        # drawing debug lines to show the dodge point and final target (which differs due to the adjustment)
        agent.line(agent.me.location, self.dodge_point, agent.renderer.white())
        agent.line(self.dodge_point-Vector(z=100), self.dodge_point+Vector(z=100), agent.renderer.red())
        agent.line(final_target-Vector(z=100), final_target+Vector(z=100), agent.renderer.green())

        defaultPD(agent, local_final_target, direction)
        defaultThrottle(agent, speed_required, direction)

        agent.line(agent.me.location, agent.me.location + (self.shot_vector * agent.best_shot_value), agent.renderer.white())

        agent.controller.handbrake = (angle_to_target >= 2.3 or (agent.me.local(agent.me.velocity).x >= 900 and angle_to_target > 1.57)) and direction == 1

        if not self.jumping:
            velocity = 1+agent.me.velocity.magnitude()
            if raw_time_remaining <= 0 or (speed_required - 2300) * time_remaining > 45 or not shot_valid(agent, self):
                # If we're out of time or not fast enough to be within 45 units of target at the intercept time, we pop
                agent.pop()
                agent.shooting = False
                agent.shot_weight = -1
                agent.shot_time = -1
                if agent.me.airborne:
                    agent.push(recovery())
            elif local_acceleration_required.z > jump_threshold and local_acceleration_required.z > local_acceleration_required.flatten().magnitude():
                # Switch into the jump when the upward acceleration required reaches our threshold, and our lateral acceleration is negligible
                self.jumping = True
            elif angle_to_target < 0.05 and distance_remaining > 2560 and velocity > 600 and velocity < speed_required - 150 and distance_remaining / velocity > 2:
                agent.push(flip(local_final_target))
            elif angle_to_target >= 1.57 and distance_remaining > 1000 and velocity < 200 and distance_remaining / velocity > 2:
                agent.push(flip(local_final_target, True))
            elif agent.me.airborne:
                agent.push(recovery(local_final_target))
        else:
            if (raw_time_remaining > 0.2 and not shot_valid(agent, self, 150)) or raw_time_remaining <= -0.9 or (not agent.me.airborne and self.counter > 0):
                agent.pop()
                agent.shooting = False
                agent.shot_weight = -1
                agent.shot_time = -1
                agent.push(recovery())
            elif self.counter == 0 and local_acceleration_required.z > 0 and raw_time_remaining > 0.083:
                # Initial jump to get airborne + we hold the jump button for extra power as required
                agent.controller.jump = True
            elif self.counter < 3:
                # make sure we aren't jumping for at least 3 frames
                agent.controller.jump = False
                self.counter += 1
            elif raw_time_remaining <= 0.1 and raw_time_remaining > -0.9:
                # dodge in the direction of the shot_vector
                agent.controller.jump = True
                if not self.dodging:
                    vector = agent.me.local(self.shot_vector)
                    self.p = abs(vector.x) * -sign(vector.x)
                    self.y = abs(vector.y) * sign(vector.y) * direction
                    self.dodging = True
                # simulating a deadzone so that the dodge is more natural
                agent.controller.pitch = self.p if abs(self.p) > 0.2 else 0
                agent.controller.yaw = self.y if abs(self.y) > 0.3 else 0


class generic_kickoff:
    def __init__(self):
        self.flip = False

    def run(self, agent):
        if self.flip:
            agent.kickoff_done = True
            agent.pop()
            return

        target = agent.ball.location + Vector(y=200*side(agent.team))
        local_target = agent.me.local(target - agent.me.location)

        defaultPD(agent, local_target)
        agent.controller.throttle = 1
        agent.controller.boost = True

        distance = local_target.magnitude()

        if distance < 550:
            self.flip = True
            agent.push(flip(agent.me.local(agent.foe_goal.location - agent.me.location)))


class corner_kickoff:
    def __init__(self, direction):
        self.direction = direction
        self.start_time = None
        self.drive_right = None
        self.last_boost = 34
        self.flip = None
        self.flip_done = None

    def run(self, agent):
        if self.start_time is None:
            self.start_time = agent.time

        agent.controller.throttle = 1

        if agent.me.boost > 12:
            agent.controller.boost = True

        if agent.me.boost > self.last_boost:
            if self.drive_right is None:
                self.drive_right = agent.time

        if self.drive_right is not None:
            if agent.time - self.drive_right < 0.1:
                agent.controller.steer = self.direction
            elif self.flip_done is None:
                if self.flip is None:
                    self.flip = flip(agent.me.local(agent.ball.location + Vector(y=1400 * side(agent.team)) - agent.me.location))
                self.flip_done = self.flip.run(agent)
            else:
                agent.kickoff_done = True
                agent.pop()

        self.last_boost = agent.me.boost

        if agent.time - self.start_time > 3:  # fine tune this value
            agent.kickoff_done = True
            agent.pop()


class recovery:
    # Point towards our velocity vector and land upright, unless we aren't moving very fast
    # A vector can be provided to control where the car points when it lands
    def __init__(self, target=None):
        self.target = target

    def run(self, agent):
        local_target = agent.me.local(agent.me.velocity.flatten()) if self.target is None else agent.me.local((self.target - agent.me.location).flatten())

        agent.dbg_2d(f"Recovering towards {self.target}")
        recover_on_ceiling = agent.me.velocity.z > 0 and agent.me.location.z > 1500
        if recover_on_ceiling:
            agent.dbg_2d(f"Recovering on the ceiling")

        defaultPD(agent, local_target, upside_down=recover_on_ceiling)
        agent.controller.throttle = 1
        if not agent.me.airborne:
            agent.pop()


class ball_recovery:
    def __init__(self):
        self.recovery = recovery()

    def run(self, agent):
        self.recovery.target = agent.ball.location
        self.recovery.run(agent)


class short_shot:
    # This routine drives towards the ball and attempts to hit it towards a given target
    # It does not require ball prediction and kinda guesses at where the ball will be on its own
    def __init__(self, target):
        self.target = target
        self.start_time = None

    def run(self, agent):
        agent.shooting = True
        agent.shot_weight = -1

        if self.start_time is None:
            self.start_time = agent.time

        car_to_ball, distance = (agent.ball.location - agent.me.location).normalize(True)
        ball_to_target = (self.target - agent.ball.location).normalize()

        relative_velocity = car_to_ball.dot(agent.me.velocity-agent.ball.velocity)
        if relative_velocity != 0:
            eta = cap(distance / cap(relative_velocity, 400, 2300), 0, 1.5)
        else:
            eta = 1.5

        # If we are approaching the ball from the wrong side the car will try to only hit the very edge of the ball
        left_vector = car_to_ball.cross(Vector(z=1))
        right_vector = car_to_ball.cross(Vector(z=-1))
        target_vector = -ball_to_target.clamp(left_vector, right_vector)
        final_target = agent.ball.location + (target_vector*(distance/2))

        # Some adjustment to the final target to ensure we don't try to drive through any goalposts to reach it
        if abs(agent.me.location.y) > 5130:
            final_target.x = cap(final_target.x, -750, 750)

        agent.line(final_target-Vector(z=100), final_target + Vector(z=100), [255, 255, 255])

        angles = defaultPD(agent, agent.me.local(final_target-agent.me.location))
        defaultThrottle(agent, 2300 if distance > 1600 else 2300-cap(1600*abs(angles[1]), 0, 2050))
        agent.controller.boost = False
        agent.controller.handbrake = True if abs(angles[1]) >= 2.3 or (agent.me.local(agent.me.velocity).x >= 900 and abs(angles[1]) > 1.57) else agent.controller.handbrake

        if abs(angles[1]) < 0.05 and (eta < 0.45 or distance < 150):
            agent.pop()
            agent.shooting = False
            agent.shot_weight = -1
            agent.shot_time = -1
            agent.push(flip(agent.me.local(car_to_ball)))
        elif agent.time - self.start_time > 0.24:  # This will run for 3 ticks, then pop
            agent.pop()
            agent.shooting = False
            agent.shot_weight = -1
            agent.shot_time = -1


class block_ground_shot:
    def __init__(self):
        self.ball_location = None
        self.intercept_time = None
        self.direction = None
        self.brake = None

    def run(self, agent):
        agent.shooting = True
        agent.shot_weight = agent.max_shot_weight - 1

        if self.ball_location is None:
            self.ball_location, self.intercept_time, self.direction = self.get_intercept(agent)

        t = self.intercept_time - agent.time

        if self.ball_location is None or not shot_valid(agent, self, threshold=75) or t < -0.1:
            agent.shooting = False
            agent.shot_weight = -1
            agent.shot_time = -1
            agent.pop()
            return

        agent.shot_time = self.intercept_time

        if self.brake:
            if agent.ball.location.dist(agent.me.location) < 500 and agent.ball.location.y * side(agent.team) + 10 < agent.me.location.y and agent.ball.location.z < 190:
                agent.push(flip(agent.me.local(agent.ball.location)))
                agent.pop()
                return

            self.brake.run(agent)
        else:
            # current velocity
            u = agent.me.local(agent.me.velocity).x
            a = brake_accel.x
            # calculate how much distance we need to slow down
            x = (u ** 2 * -1) / (2 * a)

            if self.ball_location.dist(agent.me.location) <= x:
                self.brake = brake()
                self.brake.run(agent)
                return

            agent.line(self.ball_location.flatten(), self.ball_location.flatten() + Vector(z=250), color=[255, 0, 255])
            angles = defaultPD(agent, agent.me.local(self.ball_location - agent.me.location), self.direction)
            # We want to get there before the ball does, so take the time we have and get 3 fifths of it
            required_speed = cap(agent.me.location.dist(self.ball_location) / ((self.intercept_time - agent.time) * (3/5)), 600, 2275)
            defaultThrottle(agent, required_speed, self.direction)

            agent.controller.boost = False if abs(angles[1]) > 0.3 else agent.controller.boost
            agent.controller.handbrake = True if abs(angles[1]) >= 2.3 or (agent.me.local(agent.me.velocity).x >= 900 and abs(angles[1]) > 1.57) and self.direction == 1 else False

    def is_viable(self, agent):
        self.ball_location, self.intercept_time, self.direction = self.get_intercept(agent)
        if self.ball_location is None:
            return False

        agent.print(f"Block ground shot {round(self.ball_location.dist(agent.me.location), 4)}uu's away in {round(self.intercept_time - agent.time, 4)}s (Direction: {self.direction})")
        return True

    def get_intercept(self, agent):
        struct = agent.predictions['ball_struct']
        intercepts = []

        i = 18  # Begin by looking 0.3 seconds into the future
        while i < struct.num_slices:
            intercept_time = struct.slices[i].game_seconds
            time_remaining = intercept_time - agent.time

            ball_location = Vector(struct.slices[i].physics.location.x, struct.slices[i].physics.location.y, struct.slices[i].physics.location.z)

            if abs(ball_location.y) > 5212 or time_remaining > 3:
                break

            last_ball_location = Vector(struct.slices[i-2].physics.location.x, struct.slices[i].physics.location.y, struct.slices[i].physics.location.z)
            ball_velocity = Vector(struct.slices[i].physics.velocity.x, struct.slices[i].physics.velocity.y, struct.slices[i].physics.velocity.z).magnitude()

            i += 15 - cap(int(ball_velocity//150), 0, 13)

            if time_remaining < 0 or ball_location.z > 120 or ball_location.flat_dist(agent.friend_goal.location) > last_ball_location.flat_dist(agent.friend_goal.location):
                continue

            car_to_ball = ball_location - agent.me.location
            direction, distance = car_to_ball.normalize(True)

            forward_angle = direction.angle(agent.me.forward)
            backward_angle = math.pi - forward_angle

            forward_time = time_remaining - (forward_angle * 0.318)
            backward_time = time_remaining - (backward_angle * 0.418)

            forward_flag = forward_time > 0 and (distance / forward_time) < 1800
            backward_flag = distance < 1500 and backward_time > 0 and (distance / backward_time) < 1200

            if forward_flag or backward_flag:
                intercepts.append((
                    ball_location,
                    intercept_time,
                    1 if forward_flag else -1
                ))

        if len(intercepts) == 0:
            return None, None, None

        intercepts = list(filter(lambda i: i[1] - agent.time < 3, intercepts))

        intercepts.sort(key=lambda i: agent.me.location.dist(i[0]))

        final = (None, None, None)
        last_speed = math.inf

        for intercept in intercepts:
            speed = agent.me.location.dist(intercept[0]) / ((intercept[1] / agent.time) * (3/5))
            if abs(speed - 1100) < abs(last_speed - 1100):
                final = intercept
                last_speed = speed

        return final


class boost_upwards:
    def __init__(self):
        self.jump_time = -1
        self.jumping = True
        self.counter = 0

    def run(self, agent):
        if not self.jumping and not agent.me.airborne:
            agent.pop()

        if self.jumping:
            agent.dbg_2d("Jumping")
            if self.jump_time == -1:
                jump_elapsed = 0
                self.jump_time = agent.time
            else:
                jump_elapsed = agent.time - self.jump_time

            if jump_elapsed < jump_max_duration:
                agent.controller.jump = True
            elif jump_elapsed >= jump_max_duration and self.counter < 3:
                agent.controller.jump = False
                self.counter += 1
            elif jump_elapsed < 0.3:
                agent.controller.jump = True
            else:
                self.jumping = jump_elapsed <= 0.3
        elif agent.me.location.z < (2044 - (agent.me.velocity.z + agent.gravity.z)) * 0.8:
            agent.dbg_2d("Facing")
            target = agent.ball.location.flatten().normalize()
            target.x, target.y, target.z = cap(target.x, -0.1, 0.1), cap(target.y, -0.1, 0.1), 1
            target = agent.me.local(target)
            defaultPD(agent, target, upside_down=True)
            if agent.me.local(agent.me.forward).angle3D(target) < 0.3:
                agent.dbg_2d("Boosting")
                agent.controller.boost = True
        else:
            agent.pop()
            agent.push(ball_recovery())


class ceiling_shot:
    def __init__(self):
        self.wait_target = Vector()
        self.target_location = None
        # Routine states
        self.collect_boost = True
        self.wait = False
        self.go_high = False
        self.falling = False

    def run(self, agent):
        if self.collect_boost:
            agent.dbg_2d("Collecting boost")
            if agent.me.boost > 80:
                self.collect_boost = False
                self.wait = True
                return

            large_boosts = (boost for boost in agent.boosts if boost.large and boost.active and almost_equals(boost.location.y, 0, 100))
            closest = peek_generator(large_boosts)

            if closest is not None:
                closest_distance = closest.location.flat_dist(agent.me.location)

                for item in large_boosts:
                    item_distance = item.location.flat_dist(agent.me.location)
                    if item_distance is closest_distance:
                        if item.location.flat_dist(agent.me.location) < closest.location.flat_dist(agent.me.location):
                            closest = item
                            closest_distance = item_distance
                    elif item_distance < closest_distance:
                        closest = item
                        closest_distance = item_distance

                self.wait_target = Vector(4096, 0, 1533) if Vector(4096, 0, 1533).dist(agent.me.location) < Vector(-4096, 0, 1533).dist(agent.me.location) else Vector(-4096, 0, 1533)
                agent.push(goto_boost(closest))
        elif self.wait:
            agent.dbg_2d("Waiting")
            if agent.playstyle is not agent.playstyles.Fancy:
                agent.pop()
            elif agent.me.location.dist(self.wait_target) > 150:
                local_target = agent.me.local(self.wait_target - agent.me.location)
                angle_to_target = abs(Vector(x=1).angle(local_target))
                direction = -1 if agent.me.location.z > 100 and angle_to_target >= 2.2 else 1

                defaultPD(agent, local_target, direction)
                defaultThrottle(agent, 1400, direction)
                agent.controller.boost = False
                agent.controller.handbrake = (angle_to_target >= 2.3 or (agent.me.local(agent.me.velocity).x >= 900 and angle_to_target > 1.57)) and direction == 1
            elif agent.odd_tick % 2 == 0:
                self.target_location = valid_ceiling_shot(agent)
                if self.target_location is not None:
                    self.target_location.z = 2044
                    self.wait = False
                    self.go_high = True
        elif self.go_high:
            agent.dbg_2d("Going high")
            if agent.me.airborne:
                self.go_high = False
                self.falling = True
            else:
                local_target = agent.me.local(self.target_location - agent.me.location)
                angle_to_target = abs(Vector(x=1).angle(local_target))

                defaultPD(agent, local_target)
                defaultThrottle(agent, 2300 if self.target_location.dist(agent.me.location) > 2560 else 1400)
                agent.controller.handbrake = (angle_to_target >= 2.3 or (agent.me.local(agent.me.velocity).x >= 900 and angle_to_target > 1.57))
        elif self.falling:
            agent.dbg_2d("Falling & finding a shot")
            agent.airbud = True
            shot = agent.get_shot(agent.best_shot)

            if shot is None:
                shot = agent.get_shot((agent.foe_goal.left_post, agent.foe_goal.right_post))

                if shot is None:
                    shot = agent.get_shot()

                    if shot is None and agent.me.location.z < 642:
                        agent.pop()
            agent.airbud = False

            if shot is not None:
                agent.pop()
                agent.shoot_from(shot)
