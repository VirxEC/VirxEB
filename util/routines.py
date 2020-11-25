import virxrlcu

from util.agent import VirxERLU
from util.utils import (Vector, almost_equals, cap, cap_in_field, defaultDrive,
                        defaultPD, lerp, math, peek_generator, side, sign,
                        valid_ceiling_shot, dodge_impulse)

dt = 1/120
max_speed = 2300
throttle_accel = 66 + (2/3)
brake_accel = Vector(x=-3500)
boost_per_second = 33 + (1/3)
min_boost_time = 1/30
jump_max_duration = 0.2
jump_speed = 291 + (2/3)
jump_acc = 1458 + (1/3)


class wave_dash:
    def __init__(self, target=None):
        self.step = -1
        # 0 = forward, 1 = right, 2 = backwards, 3 = left
        self.direction = 0
        self.start_time = -1
        self.target = target

    def run(self, agent: VirxERLU):
        if self.start_time == -1:
            self.start_time = agent.time
        T = agent.time - self.start_time

        self.step += 1

        if self.target is not None:
            self.direction = 0 if abs(self.target.x) > abs(self.target.y) else 1

            if (self.direction == 0 and self.target.x < 0) or (self.direction == 1 and self.target.y < 0):
                self.direction += 2

        forward_target = agent.me.velocity.flatten().normalize() * 100

        target_switch = {
            0: forward_target + Vector(z=50),
            1: forward_target,
            2: forward_target - Vector(z=50),
            3: forward_target
        }

        target_up = {
            0: Vector(z=1),
            1: Vector(y=-50, z=1),
            2: Vector(z=1),
            3: Vector(y=50, z=1)
        }

        defaultPD(agent, agent.me.local(target_switch[self.direction]), up=agent.me.local(target_up[self.direction]))
        if self.direction == 0:
            agent.controller.throttle = 1
        elif self.direction == 2:
            agent.controller.throttle = -1
        else:
            agent.controller.handbrake = True

        if self.step < 1:
            agent.controller.jump = True
        elif self.step < 4:
            pass
        elif not agent.me.airborne:
            agent.pop()
        elif T > 2:
            agent.pop()
            agent.push(recovery(self.target))
        elif agent.me.location.z + (agent.me.velocity.z * 0.2) < 5:
            agent.controller.jump = True
            agent.controller.yaw = 0
            if self.direction in {0, 2}:
                agent.controller.roll = 0
                agent.controller.pitch = -1 if self.direction is 0 else 1
            else:
                agent.controller.roll = -1 if self.direction is 1 else 1
                agent.controller.pitch = 0


class double_jump:
    # Hits a target point at a target time towards a target direction
    def __init__(self, intercept_time, targets=None):
        self.ball_location = None
        self.shot_vector = None
        self.offset_target = None
        self.intercept_time = intercept_time
        self.targets = targets
        # Flags for what part of the routine we are in
        self.jumping = False
        self.dodged = False
        self.jump_time = -1
        self.needed_jump_time = -1
        self.counter = 0

    def update(self, shot):
        self.intercept_time = shot.intercept_time
        self.targets = shot.targets

    def run(self, agent: VirxERLU):
        # This routine is the same as jump_shot, but it's designed to hit the ball above 300uus and below 450uus without requiring boost
        if not agent.shooting:
            agent.shooting = True

        T = self.intercept_time - agent.time
        # Capping T above 0 to prevent division problems
        time_remaining = cap(T, 0.000001, 6)

        if not self.jumping or self.ball_location is None:
            slice_n = round(T * 60) - 1
            agent.dbg_2d(f"Shot slice #: {slice_n}")

            ball = agent.ball_prediction_struct.slices[slice_n].physics.location
            self.ball_location = Vector(ball.x, ball.y, ball.z)
            self.needed_jump_time = virxrlcu.get_double_jump_time(ball.z - agent.me.location.z, agent.me.velocity.z, agent.gravity.z)

        direction = (self.ball_location - agent.me.location).normalize()
        self.shot_vector = direction if self.targets is None else direction.clamp2D((self.targets[0] - self.ball_location).normalize(), (self.targets[1] - self.ball_location).normalize())
        self.offset_target = self.ball_location - (self.shot_vector * agent.best_shot_value)
        agent.sphere(self.ball_location, 92.75)

        car_to_ball = self.ball_location - agent.me.location
        final_target = cap_in_field(agent, self.offset_target.copy())  # Some adjustment to the final target to ensure it's inside the field and we don't try to drive through any goalposts or walls to reach it
        Tj = T - self.needed_jump_time * 1.15
        agent.dbg_2d(f"Needed jump time: {round(self.needed_jump_time, 2)}")

        if Tj > 0 and self.targets is not None:
            # whether we are to the left or right of the shot vector
            side_of_shot = sign(self.shot_vector.cross(Vector(z=1)).dot(car_to_ball))
            car_to_offset_target = final_target - agent.me.location
            car_to_dodge_perp = car_to_offset_target.cross(Vector(z=side_of_shot))  # perpendicular

            # The adjustment causes the car to circle around the dodge point in an effort to line up with the shot vector
            # The adjustment slowly decreases to 0 as the bot nears the time to jump
            adjustment = car_to_offset_target.angle2D(self.shot_vector) * Tj * 500  # size of adjustment
            # we don't adjust the final target if we are already jumping
            final_target += car_to_dodge_perp.normalize() * adjustment

        distance_remaining = final_target.flat_dist(agent.me.location)

        # Some adjustment to the final target to ensure it's inside the field and we don't try to drive through any goalposts or walls to reach it
        final_target = cap_in_field(agent, final_target).flatten()
        local_final_target = agent.me.local_location(final_target)

        # whether we should go forwards or backwards
        angle_to_target = abs(Vector(x=1).angle2D(local_final_target))
        direction = 1 if angle_to_target < 1.6 or agent.me.local_velocity().x > 1000 else -1

        # drawing debug lines to show the dodge point and final target (which differs due to the adjustment)
        agent.line(agent.me.location, self.offset_target, agent.renderer.white())
        agent.line(self.offset_target-Vector(z=100), self.offset_target+Vector(z=100), agent.renderer.green())
        agent.line(final_target-Vector(z=100), final_target+Vector(z=100), agent.renderer.purple())

        vf = agent.me.velocity + agent.gravity * T

        distance_remaining = agent.me.local_location(self.offset_target).x if agent.me.airborne else distance_remaining
        distance_remaining -= agent.me.hitbox.length / 3
        speed_required = distance_remaining / time_remaining
        agent.dbg_2d(f"Speed required: {round(speed_required, 2)}")

        if speed_required < 1900 and agent.me.boost > 50 and T > 2 and distance_remaining > 2560:
            agent.dbg_2d("Building speed")
            speed_required *= 0.75

        if not self.jumping:
            velocity = defaultDrive(agent, speed_required * direction, local_final_target)[1]
            if velocity == 0: velocity = 1

            local_offset_target = agent.me.local_location(self.offset_target.flatten())
            local_vf = agent.me.local(vf.flatten())

            if ((abs(velocity) < 100 and distance_remaining < agent.me.hitbox.length / 2) or (abs(local_offset_target.y) < 92 and direction * local_vf.x >= direction * (local_offset_target.x - agent.me.hitbox.length / 3) and direction * local_offset_target.x > 0)) and T <= self.needed_jump_time * 1.025:
                self.jumping = True
            elif agent.me.airborne:
                agent.push(recovery(local_final_target if Tj > 0 else None))
            elif T <= 0 or (Tj > 0 and distance_remaining > 50 and not virxrlcu.double_jump_shot_is_viable(T, agent.boost_accel, tuple(agent.gravity), agent.me.get_raw(agent), self.offset_target.z, tuple((final_target - agent.me.location).normalize()), distance_remaining)):
                # If we're out of time or the ball was hit away or we just can't get enough speed, pop
                agent.pop()
                agent.shooting = False
                agent.shot_weight = -1
                agent.shot_time = -1
                if agent.me.airborne:
                    agent.push(ball_recovery())
            elif agent.boost_amount != 'unlimited' and agent.me.boost < 36 and angle_to_target < 0.03 and velocity > 600 and velocity + dodge_impulse(agent) <= speed_required and distance_remaining / velocity > 3:
                if agent.gravity.z < -450 and distance_remaining / velocity < 5:
                    agent.push(wave_dash(agent.me.local_location(self.offset_target)))
                else:
                    agent.push(flip(agent.me.local_location(self.offset_target)))
            elif agent.boost_amount != 'unlimited' and angle_to_target >= 2 and distance_remaining > 2560 and velocity < 200:
                agent.push(flip(agent.me.local_location(self.offset_target), True))
        else:
            # Mark the time we started jumping so we know when to dodge
            if self.jump_time == -1:
                self.jump_time = agent.time

            jump_elapsed = agent.time - self.jump_time
            tau = jump_max_duration - jump_elapsed

            xf = agent.me.location + agent.me.velocity * T + 0.5 * agent.gravity * T * T

            if jump_elapsed == 0:
                vf += agent.me.up * jump_speed
                xf += agent.me.up * jump_speed * T

            hf = vf
            vf += agent.me.up * jump_acc * tau
            xf += agent.me.up * jump_acc * tau * (T - 0.5 * tau)

            hf += agent.me.up * jump_speed
            vf += agent.me.up * jump_speed
            xf += agent.me.up * jump_speed * (T - tau)

            delta_x = self.offset_target - xf
            d_direction = delta_x.normalize()

            if abs(agent.me.forward.dot(d_direction)) > 0.5:
                delta_v = delta_x.dot(agent.me.forward) / T
                if agent.me.boost > 0 and delta_v >= agent.boost_accel * min_boost_time:
                    agent.controller.boost = True
                else:
                    agent.controller.throttle = cap(delta_v / (throttle_accel * min_boost_time), -1, 1)

            if T <= -0.4 or (not agent.me.airborne and self.counter > 0):
                agent.pop()
                agent.shooting = False
                agent.shot_weight = -1
                agent.shot_time = -1
                agent.push(ball_recovery())
            elif jump_elapsed < jump_max_duration and hf.z <= self.offset_target.z:
                agent.controller.jump = True
            elif self.counter < 4:
                self.counter += 1

            if self.counter == 3:
                agent.controller.jump = True
            elif self.counter == 4:
                defaultPD(agent, agent.me.local(self.shot_vector * direction), upside_down=True)

            if self.counter < 3:
                defaultPD(agent, agent.me.local(self.shot_vector.flatten() * direction))

        l_vf = vf + agent.me.location
        agent.line(l_vf-Vector(z=100), l_vf+Vector(z=100), agent.renderer.red())


class Aerial:
    def __init__(self, intercept_time, targets=None, fast_aerial=True):
        self.intercept_time = intercept_time
        self.fast_aerial = fast_aerial
        self.targets = targets
        self.shot_vector = None
        self.target = None
        self.ball = None

        self.jump_type_fast = None
        self.jumping = False
        self.dodging = False
        self.ceiling = False
        self.jump_time = -1
        self.counter = 0

    def update(self, shot):
        self.intercept_time = shot.intercept_time
        self.fast_aerial = shot.fast_aerial
        self.targets = shot.targets

    def run(self, agent: VirxERLU):
        if not agent.shooting:
            agent.shooting = True

        T = self.intercept_time - agent.time
        xf = agent.me.location + agent.me.velocity * T + 0.5 * agent.gravity * T * T
        vf = agent.me.velocity + agent.gravity * T

        slice_n = math.ceil(T * 60) - 1
        agent.dbg_2d(f"Shot slice #: {slice_n}")

        if T > 0.1 or self.ball is None:
            ball = agent.ball_prediction_struct.slices[slice_n].physics.location
            self.ball = Vector(ball.x, ball.y, ball.z)
            self.ceiling = agent.me.location.z > 2044 - agent.me.hitbox.height * 2 and not agent.me.jumped

        direction = (agent.ball.location - agent.me.location).normalize()
        self.shot_vector = direction if self.targets is None else direction.clamp((self.targets[0] - self.ball).normalize(), (self.targets[1] - self.ball).normalize())
        self.target = self.ball - (self.shot_vector * agent.best_shot_value)
        agent.sphere(self.ball, 92.75)

        if self.ceiling:
            self.target -= Vector(z=92)

        if self.jumping or not agent.me.airborne:
            agent.dbg_2d("Jumping")

            if not self.jumping or not agent.me.airborne:
                self.jump_type_fast = self.fast_aerial
                self.jumping = True
                self.jump_time = agent.time
                self.counter = 0

            jump_elapsed = agent.time - self.jump_time

            # how much of the jump acceleration time is left
            tau = jump_max_duration - jump_elapsed

            # impulse from the first jump
            if jump_elapsed == 0:
                vf += agent.me.up * jump_speed
                xf += agent.me.up * jump_speed * T

            # acceleration from holding jump
            vf += agent.me.up * jump_acc * tau
            xf += agent.me.up * jump_acc * tau * (T - 0.5 * tau)

            if self.jump_type_fast:
                # impulse from the second jump
                vf += agent.me.up * jump_speed
                xf += agent.me.up * jump_speed * (T - tau)

                if jump_elapsed <= jump_max_duration:
                    agent.controller.jump = True
                elif self.counter < 4:
                    self.counter += 1

                if self.counter == 3:
                    agent.controller.jump = True
                    self.dodging = True
                elif self.counter == 4:
                    self.dodging = self.jumping = False
            elif jump_elapsed <= jump_max_duration:
                agent.controller.jump = True
            else:
                self.jumping = False

        if self.ceiling:
            agent.dbg_2d(f"Ceiling shot")

        delta_x = self.target - xf
        direction = delta_x.normalize() if not self.jumping else delta_x.flatten().normalize()

        agent.line(agent.me.location, agent.me.location + (direction * 250), agent.renderer.black())
        c_vf = vf + agent.me.location
        agent.line(c_vf - Vector(z=100), c_vf + Vector(z=100), agent.renderer.blue())
        agent.line(xf - Vector(z=100), xf + Vector(z=100), agent.renderer.red())
        agent.line(self.target - Vector(z=100), self.target + Vector(z=100), agent.renderer.green())

        if not self.dodging:
            target = delta_x if delta_x.magnitude() > 50 else self.shot_vector

            target = agent.me.local(target)
            land_on_ceiling = virxrlcu.find_landing_plane(tuple(agent.me.location), tuple(agent.me.velocity), agent.gravity.z) == 4

            if self.jumping:
                defaultPD(agent, target.flatten(), up=agent.me.up)
            elif land_on_ceiling:
                defaultPD(agent, target, upside_down=True)
            else:
                defaultPD(agent, target, upside_down=self.shot_vector.z < 0)
                # abs(Vector(x=1).angle(target)) < 0.25
                # abs(agent.me.orientation.det() - 1.0) < 0.01
                if T > 1:
                    agent.controller.roll = 1 if self.shot_vector.z < 0 else -1

        # only boost/throttle if we're facing the right direction
        if abs(agent.me.forward.dot(direction)) > 0.5 and T > 0:
            # the change in velocity the bot needs to put it on an intercept course with the target
            delta_v = delta_x.dot(agent.me.forward) / T
            if agent.me.boost > 0 and delta_v >= agent.boost_accel * min_boost_time:
                agent.controller.boost = True
                delta_v -= agent.boost_accel * min_boost_time

            if delta_v >= throttle_accel * min_boost_time:
                agent.controller.throttle = cap(delta_v / (throttle_accel * min_boost_time), -1, 1)

        if T <= -0.4 or (not self.jumping and T > 1.5 and not virxrlcu.aerial_shot_is_viable(T, agent.boost_accel, tuple(agent.gravity), agent.me.get_raw(agent), tuple(self.target))):
            agent.pop()
            agent.shooting = False
            agent.shot_weight = -1
            agent.shot_time = -1
            agent.push(ball_recovery())
        elif (self.ceiling and self.target.dist(agent.me.location) < 92 + agent.me.hitbox.length and not agent.me.doublejumped and agent.me.location.z < agent.ball.location.z + 92 and self.target.y * side(agent.team) > -4240) or (not self.ceiling and not agent.me.doublejumped and T < 0.1):
            agent.dbg_2d("Flipping")
            local_target = agent.me.local(self.shot_vector).flatten().normalize()
            agent.controller.pitch = -local_target.x
            agent.controller.yaw = local_target.y
            agent.controller.jump = True


class flip:
    # Flip takes a vector in local coordinates and flips/dodges in that direction
    # cancel causes the flip to cancel halfway through, which can be used to half-flip
    def __init__(self, vector, cancel=False):
        vector = vector.flatten().normalize()
        self.pitch = -vector.x
        self.yaw = vector.y
        self.cancel = cancel
        # the time the jump began
        self.time = -1
        # keeps track of the frames the jump button has been released
        self.counter = 0

    def run(self, agent: VirxERLU, manual=False):
        if self.time == -1:
            elapsed = 0
            self.time = agent.time
        else:
            elapsed = agent.time - self.time

        if elapsed < 0.075:
            agent.controller.jump = True
        elif elapsed >= 0.075 and self.counter < 3:
            agent.controller.jump = False
            self.counter += 1
        elif elapsed < 0.4 or (not self.cancel and elapsed < 0.9):
            agent.controller.jump = True
            agent.controller.pitch = self.pitch
            agent.controller.yaw = self.yaw
        else:
            if not manual:
                agent.pop()
            agent.push(recovery())
            return True


class brake:
    @staticmethod
    def run(agent: VirxERLU, manual=False):
        # current forward velocity
        speed = agent.me.local_velocity().x
        if abs(speed) > 100:
            # apply our throttle in the opposite direction
            agent.controller.throttle = -cap(speed / throttle_accel, -1, 1)
        elif not manual:
            agent.pop()


class goto:
    # Drives towards a designated (stationary) target
    # Optional vector controls where the car should be pointing upon reaching the target
    def __init__(self, target, vector=None, brake=False):
        self.target = target
        self.vector = vector
        self.brake = brake

        self.f_brake = False
        self.rule1_timer = -1

    def run(self, agent: VirxERLU, manual=False):
        car_to_target = self.target - agent.me.location
        distance_remaining = car_to_target.flatten().magnitude()

        agent.dbg_2d(f"Distance to target: {round(distance_remaining)}")
        agent.line(self.target - Vector(z=500), self.target + Vector(z=500), (255, 0, 255))

        if self.brake and (self.f_brake or distance_remaining * 0.95 < (agent.me.local_velocity().x ** 2 * -1) / (2 * brake_accel.x)):
            self.f_brake = True
            brake.run(agent, manual=manual)
            return

        if not self.brake and not manual and distance_remaining < 640:
            agent.pop()
            return

        final_target = self.target.copy()

        # Some adjustment to the final target to ensure it's inside the field and we don't try to drive through any goalposts to reach it
        final_target = cap_in_field(agent, final_target)

        if self.vector is not None:
            # See comments for adjustment in jump_shot for explanation
            side_of_vector = sign(self.vector.cross(Vector(z=1)).dot(car_to_target))
            car_to_target_perp = car_to_target.cross(Vector(z=side_of_vector)).normalize()
            adjustment = car_to_target.angle2D(self.vector) * distance_remaining / 3.14
            final_target += car_to_target_perp * adjustment

        final_target = cap_in_field(agent, final_target)
        local_target = agent.me.local_location(final_target)
        angle_to_target = abs(Vector(x=1).angle2D(local_target))
        direction = 1 if angle_to_target < 1.6 or agent.me.local_velocity().x > 1000 else -1
        agent.dbg_2d(f"Angle to target: {round(angle_to_target, 1)}")

        velocity = defaultDrive(agent, 2300 * direction, local_target)[1]
        if distance_remaining < 1280: agent.controller.boost = False
        if velocity == 0: velocity = 1

        # this is to break rule 1's with TM8'S ONLY
        # 251 is the distance between center of the 2 longest cars in the game, with a bit extra
        if len(agent.friends) > 0 and agent.me.local_velocity().x < 50 and agent.controller.throttle == 1 and min(agent.me.location.flat_dist(car.location) for car in agent.friends) < 251:
            if self.rule1_timer == -1:
                self.rule1_timer = agent.time
            elif agent.time - self.rule1_timer > 1.5:
                agent.push(flip(Vector(y=250)))
                return
        elif self.rule1_timer != -1:
            self.rule1_timer = -1

        if agent.me.airborne:
            agent.push(recovery(self.target))
        elif agent.me.boost < 60 and angle_to_target < 0.03 and velocity > 500 and velocity < 2150 and distance_remaining / velocity > 2:
            agent.push(flip(agent.me.local_location(self.target)))
        elif direction == -1 and distance_remaining > 1000 and velocity < 200:
            agent.push(flip(agent.me.local_location(self.target), True))


class shadow:
    def __init__(self):
        self.goto = goto(Vector(), brake=True)

    def run(self, agent: VirxERLU):
        ball_loc = self.get_ball_loc(agent, True)

        if (agent.playstyle is agent.playstyles.Defensive and len(agent.friends) > 0) or ball_loc.y > 2560 or agent.me.location.y * side(agent.team) < ball_loc.y:
            agent.pop()
            agent.push(retreat())
            return

        target = self.get_target(agent, ball_loc)

        self_to_target = agent.me.location.flat_dist(target)

        if self_to_target < 320 and ball_loc.y < -640 and agent.me.velocity.magnitude() < 50 and abs(Vector(x=1).angle2D(agent.me.local_location(agent.ball.location))) > 1:
            agent.pop()
            agent.push(face_target(ball=True))
        else:
            self.goto.target = target
            self.goto.vector = ball_loc
            self.goto.run(agent)

    def is_viable(self, agent):
        ball_loc = self.get_ball_loc(agent)
        target = self.get_target(agent, ball_loc)
        self_to_target = agent.me.location.flat_dist(target)

        return (not ((agent.playstyle is agent.playstyles.Defensive and len(agent.friends) > 0) or ball_loc.y >= 2560 or agent.me.location.y * side(agent.team) <= ball_loc.y)) and self_to_target > 320

    def get_ball_loc(self, agent, render=False):
        ball_slice = agent.ball_prediction_struct.slices[cap(round(agent.predictions['enemy_time_to_ball'] * 0.95 * 60), 0, len(agent.ball_prediction_struct.slices) - 1)].physics.location
        ball_loc = Vector(ball_slice.x, ball_slice.y)
        if render: agent.sphere(ball_loc + Vector(z=93), 92.75, color=agent.renderer.black())
        ball_loc.y *= side(agent.team)

        if ball_loc.y < -2560 or (ball_loc.y < agent.ball.location.y * side(agent.team)):
            ball_loc = Vector(agent.ball.location.x, agent.ball.location.y * side(agent.team) + 640)

        return ball_loc

    def get_target(self, agent: VirxERLU, ball_loc=None):
        if ball_loc is None:
            ball_loc = self.get_ball_loc(agent)

        if len(agent.friends) == 1 and agent.playstyle is agent.playstyles.Neutral and (agent.predictions['was_down'] or abs(agent.game.friend_score - agent.game.foe_score) > 1):
            distance = 4480
        else:
            distance = 1920 if agent.playstyle is agent.playstyles.Defensive else 2560
        distance *= 1 - (agent.predictions['enemy_time_to_ball'] / 10)

        target = Vector(y=(ball_loc.y + distance) * side(agent.team))
        if target.y * side(agent.team) > -1280:
            # find the proper x coord for us to stop a shot going to the net
            # y = mx + b <- yes, finally! 7th grade math is paying off xD
            p1 = agent.friend_goal.location.flatten()
            p2 = ball_loc * Vector(x=1, y=side(agent.team))
            try:
                m = (p2.y - p1.y) / (p2.x - p1.x)
                b = p1.y - (m * p1.x)
                # x = (y - b) / m
                target.x = (target.y - b) / m
            except ZeroDivisionError:
                target.x = 0
        else:
            target.x = (abs(ball_loc.x) + 320) * sign(ball_loc.x)

        return Vector(cap(target.x, -4096, 4096), cap(target.y, -5120, 5120))


class retreat:
    def __init__(self):
        self.goto = goto(Vector(), brake=True)

    def run(self, agent: VirxERLU):
        ball_slice = agent.ball_prediction_struct.slices[agent.future_ball_location_slice].physics.location
        ball = Vector(ball_slice.x, cap(ball_slice.y, -5120, 5120))
        agent.line(ball, ball + Vector(z=185), agent.renderer.white())
        ball.y *= side(agent.team)

        if ball.y < agent.ball.location.y * side(agent.team):
            ball = Vector(agent.ball.location.x, agent.ball.location.y * side(agent.team) + 640)

        if ball.y < 2560 and agent.playstyle is not agent.playstyles.Defensive and agent.me.location.y * side(agent.team) > ball.y:
            agent.pop()
            agent.push(shadow())
            return

        target = self.get_target(agent)
        self_to_target = agent.me.location.flat_dist(target)

        if self_to_target < 120:
            agent.pop()

            if agent.me.local_velocity().x > throttle_accel:
                agent.push(brake())
            return

        self.goto.target = target
        self.goto.run(agent)

    def is_viable(self, agent):
        ball_slice = agent.ball_prediction_struct.slices[cap(round(agent.predictions['enemy_time_to_ball'] * 0.95 * 60), 0, len(agent.ball_prediction_struct.slices) - 1)].physics.location
        ball = Vector(ball_slice.x, cap(ball_slice.y, -5120, 5120))
        ball.y *= side(agent.team)

        if ball.y < agent.ball.location.y * side(agent.team):
            ball = Vector(agent.ball.location.x, agent.ball.location.y * side(agent.team) + 640)

        target = self.get_target(agent)
        self_to_target = agent.me.location.flat_dist(target)

        return (not (ball.y < 2560 and agent.playstyle is not agent.playstyles.Defensive and agent.me.location.y * side(agent.team) > ball.y)) and self_to_target > 320

    def get_target(self, agent: VirxERLU):
        target = None
        ball_slice = agent.ball_prediction_struct.slices[cap(round(agent.predictions['enemy_time_to_ball'] * 0.95 * 60), 0, len(agent.ball_prediction_struct.slices) - 1)].physics.location
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
        self.start_loc = None
        self.counter = 0

    def run(self, agent: VirxERLU):
        if self.ball:
            target = Vector(agent.ball.location.x, cap(agent.ball.location.y, -5120, 5120)) - agent.me.location.flatten()
        else:
            target = agent.me.velocity.flatten() if self.target is None else (self.target - agent.me.location).flatten()

        if agent.gravity.z < -550 and agent.gravity.z > -750:
            if self.counter == 0 and abs(Vector(x=1).angle(target)) <= 0.05:
                agent.pop()
                return

            if self.counter == 0 and agent.me.airborne:
                self.counter = 3

            if self.counter < 3:
                self.counter += 1

            target = agent.me.local(target)
            if self.counter < 3:
                agent.controller.jump = True
            elif agent.me.airborne and abs(Vector(x=1).angle(target)) > 0.05:
                defaultPD(agent, target)
            else:
                agent.pop()
        else:
            target = agent.me.local(target)
            angle_to_target = abs(Vector(x=1).angle2D(target))
            if angle_to_target > 0.1:
                if self.start_loc is None:
                    self.start_loc = agent.me.location

                direction = -1 if angle_to_target < 1.57 else 1

                agent.controller.steer = cap(target.y / 100, -1, 1) * direction
                agent.controller.throttle = direction
                agent.controller.handbrake = True
            else:
                agent.pop()
                if self.start_loc is not None:
                    agent.push(goto(self.start_loc, target, True))


class goto_boost:
    # very similar to goto() but designed for grabbing boost
    def __init__(self, boost):
        self.boost = boost
        self.goto = goto(self.boost.location)

    def run(self, agent: VirxERLU):
        if not self.boost.large:
            self.goto.vector = agent.ball.location

        if not self.boost.active:
            agent.pop()
        else:
            self.goto.run(agent, manual=True)


class jump_shot:
    # Hits a target point at a target time towards a target direction
    def __init__(self, intercept_time, targets=None):
        self.ball_location = None
        self.shot_vector = None
        self.offset_target = None
        self.intercept_time = intercept_time
        self.targets = targets
        # Flags for what part of the routine we are in
        self.jumping = False
        self.dodging = False
        self.counter = 0
        self.jump_time = -1
        self.needed_jump_time = -1

    def update(self, shot):
        self.intercept_time = shot.intercept_time
        self.targets = shot.targets

    def run(self, agent: VirxERLU):
        if not agent.shooting:
            agent.shooting = True

        T = self.intercept_time - agent.time
        # Capping T above 0 to prevent division problems
        time_remaining = cap(T, 0.000001, 6)

        if not self.jumping or self.ball_location is None:
            slice_n = round(T * 60) - 1
            agent.dbg_2d(f"Shot slice #: {slice_n}")

            ball = agent.ball_prediction_struct.slices[slice_n].physics.location
            self.ball_location = Vector(ball.x, ball.y, ball.z)
            self.needed_jump_time = virxrlcu.get_jump_time(ball.z - agent.me.location.z, agent.me.velocity.z, agent.gravity.z)

        direction = (self.ball_location - agent.me.location).normalize()
        self.shot_vector = direction if self.targets is None else direction.clamp2D((self.targets[0] - self.ball_location).normalize(), (self.targets[1] - self.ball_location).normalize())
        self.offset_target = self.ball_location - (self.shot_vector * agent.best_shot_value)
        agent.sphere(self.ball_location, 92.75)

        car_to_ball = self.ball_location - agent.me.location
        final_target = cap_in_field(agent, self.offset_target.copy())  # Some adjustment to the final target to ensure it's inside the field and we don't try to drive through any goalposts or walls to reach it
        Tj = T - self.needed_jump_time * 1.15
        agent.dbg_2d(f"Needed jump time: {round(self.needed_jump_time, 2)}")

        if Tj > 0 and self.targets is not None:
            # whether we are to the left or right of the shot vector
            side_of_shot = sign(self.shot_vector.cross(Vector(z=1)).dot(car_to_ball))
            car_to_offset_target = final_target - agent.me.location
            car_to_offset_perp = car_to_offset_target.cross(Vector(z=side_of_shot))  # perpendicular

            # The adjustment causes the car to circle around the dodge point in an effort to line up with the shot vector
            # The adjustment slowly decreases to 0 as the bot nears the time to jump
            adjustment = car_to_offset_target.angle2D(self.shot_vector) * Tj * 500  # size of adjustment
            final_target += car_to_offset_perp.normalize() * adjustment

        distance_remaining = final_target.flat_dist(agent.me.location)

        # Some adjustment to the final target to ensure it's inside the field and we don't try to drive through any goalposts or walls to reach it (again)
        final_target = cap_in_field(agent, final_target).flatten()
        local_final_target = agent.me.local_location(final_target)

        # whether we should go forwards or backwards
        angle_to_target = abs(Vector(x=1).angle2D(local_final_target))
        direction = 1 if angle_to_target < 1.6 or agent.me.local_velocity().x > 1000 else -1

        # drawing debug lines to show the dodge point and final target (which differs due to the adjustment)
        agent.line(agent.me.location, self.offset_target, agent.renderer.white())
        agent.line(self.offset_target-Vector(z=100), self.offset_target+Vector(z=100), agent.renderer.green())
        agent.line(final_target-Vector(z=100), final_target+Vector(z=100), agent.renderer.purple())

        vf = agent.me.velocity + agent.gravity * T

        distance_remaining = agent.me.local_location(self.offset_target).x if agent.me.airborne else distance_remaining
        distance_remaining -= agent.me.hitbox.length / 3
        speed_required = distance_remaining / time_remaining
        agent.dbg_2d(f"Speed required: {round(speed_required, 2)}")

        if speed_required < 1900 and agent.me.boost > 50 and T > 2 and distance_remaining > 2560:
            agent.dbg_2d("Building speed")
            speed_required *= 0.75

        if not self.jumping:
            agent.dbg_2d(f"jump time: {self.needed_jump_time}")

            velocity = defaultDrive(agent, speed_required * direction, local_final_target)[1]
            if velocity == 0: velocity = 1

            local_offset_target = agent.me.local_location(self.offset_target.flatten())
            local_vf = agent.me.local(vf.flatten())

            if ((abs(velocity) < 100 and distance_remaining < agent.me.hitbox.length / 2) or (abs(local_offset_target.y) < 92 and direction * local_vf.x >= direction * (local_offset_target.x - agent.me.hitbox.length / 3) and direction * local_offset_target.x > 0)) and T <= self.needed_jump_time * 1.025:
                self.jumping = True
            elif agent.me.airborne:
                agent.push(recovery(local_final_target if Tj > 0 else None))
            elif T <= 0 or (Tj > 0 and distance_remaining > 50 and not virxrlcu.jump_shot_is_viable(T, agent.boost_accel, tuple(agent.gravity), agent.me.get_raw(agent), self.offset_target.z, tuple((final_target - agent.me.location).normalize()), distance_remaining)):
                # If we're out of time or not fast enough to be within 45 units of target at the intercept time, we pop
                agent.pop()
                agent.shooting = False
                agent.shot_weight = -1
                agent.shot_time = -1
                if agent.me.airborne:
                    agent.push(recovery())
            elif agent.boost_amount != 'unlimited' and agent.me.boost < 36 and angle_to_target < 0.03 and velocity > 600 and velocity + dodge_impulse(agent) <= speed_required and distance_remaining / velocity > 3:
                if agent.gravity.z < -450 and distance_remaining / velocity < 5:
                    agent.push(wave_dash(agent.me.local_location(self.offset_target)))
                else:
                    agent.push(flip(agent.me.local_location(self.offset_target)))
            elif agent.boost_amount != 'unlimited' and direction == -1 and distance_remaining > 1500 and velocity < 200 and distance_remaining / abs(velocity) > 2:
                agent.push(flip(agent.me.local_location(self.offset_target), True))
        else:
            if self.jump_time == -1:
                self.jump_time = agent.time

            jump_elapsed = agent.time - self.jump_time
            tau = jump_max_duration - jump_elapsed

            xf = agent.me.location + agent.me.velocity * T + 0.5 * agent.gravity * T * T

            if jump_elapsed == 0:
                vf += agent.me.up * jump_speed
                xf += agent.me.up * jump_speed * T

            hf = vf.z
            vf += agent.me.up * jump_acc * tau
            xf += agent.me.up * jump_acc * tau * (T - 0.5 * tau)

            delta_x = self.offset_target - xf
            d_direction = delta_x.normalize()

            if abs(agent.me.forward.dot(d_direction)) > 0.5 and self.counter < 3:
                delta_v = delta_x.dot(agent.me.forward) / T
                if agent.me.boost > 0 and delta_v >= agent.boost_accel * min_boost_time:
                    agent.controller.boost = True
                    delta_v -= agent.boost_accel * min_boost_time

                if abs(delta_v) > throttle_accel * min_boost_time:
                    agent.controller.throttle = cap(delta_v / (throttle_accel * min_boost_time), -1, 1)

            if T <= -0.4 or (not agent.me.airborne and self.counter >= 3):
                agent.pop()
                agent.shooting = False
                agent.shot_weight = -1
                agent.shot_time = -1
                agent.push(recovery())
                return
            else:
                if self.counter == 3 and agent.me.location.dist(self.offset_target) < (92.75 + agent.me.hitbox.length) * 1.05:
                    # Get the required pitch and yaw to flip correctly
                    vector = agent.me.local_location(self.offset_target).flatten().normalize()
                    self.p = -vector.x
                    self.y = vector.y

                    agent.controller.pitch = self.p
                    agent.controller.yaw = self.y
                    # Wait 1 more frame before dodging
                    self.counter += 1
                elif self.counter == 4:
                    # Dodge
                    agent.controller.jump = True
                    agent.controller.pitch = self.p
                    agent.controller.yaw = self.y
                else:
                    # Face the target as much as possible
                    defaultPD(agent, agent.me.local_location(self.offset_target * direction))

                if jump_elapsed <= jump_max_duration and hf <= self.offset_target.z:
                    # Initial jump to get airborne + we hold the jump button for extra power as required
                    agent.controller.jump = True
                elif self.counter < 3:
                    # Make sure we aren't jumping for at least 3 frames
                    self.counter += 1

        l_vf = vf + agent.me.location
        agent.line(l_vf-Vector(z=100), l_vf+Vector(z=100), agent.renderer.red())


class ground_shot:
    # Hits a target point at a target time towards a target direction
    def __init__(self, intercept_time, targets=None):
        self.ball_location = None
        self.shot_vector = None
        self.offset_target = None
        self.intercept_time = intercept_time
        self.targets = targets

    def update(self, shot):
        self.intercept_time = shot.intercept_time
        self.targets = shot.targets

    def run(self, agent: VirxERLU):
        if not agent.shooting:
            agent.shooting = True

        T = self.intercept_time - agent.time
        # Capping T above 0 to prevent division problems
        time_remaining = cap(T, 0.000001, 6)

        if T > 0.1 or self.ball_location is None:
            slice_n = round(T * 60) - 1
            agent.dbg_2d(f"Shot slice #: {slice_n}")

            ball = agent.ball_prediction_struct.slices[slice_n].physics.location
            self.ball_location = Vector(ball.x, ball.y, ball.z)

        direction = (self.ball_location - agent.me.location).normalize()
        self.shot_vector = direction if self.targets is None else direction.clamp2D((self.targets[0] - self.ball_location).normalize(), (self.targets[1] - self.ball_location).normalize())
        self.offset_target = self.ball_location - (self.shot_vector * agent.best_shot_value)
        agent.sphere(self.ball_location, 92.75)

        l_ball = agent.me.local(agent.ball.location)
        car_to_ball = agent.ball.location - agent.me.location
        if abs(l_ball.y) < 92.75 + agent.me.hitbox.width / 2 and abs(l_ball.z) < 92.75 + agent.me.hitbox.height / 2:
            final_target = l_ball - (self.shot_vector * agent.best_shot_value)
            distance_remaining = final_target.flat_dist(agent.me.location) - agent.me.hitbox.length / 3
            speed_required = 2300
            agent.dbg_2d(f"Max speed")
        else:
            # Some adjustment to the final target to ensure it's inside the field and we don't try to drive through any goalposts or walls to reach it
            final_target = cap_in_field(agent, self.offset_target.copy())

            if self.targets is not None:
                # whether we are to the left or right of the shot vector
                side_of_shot = sign(self.shot_vector.cross(Vector(z=1)).dot(car_to_ball))
                car_to_offset_target = final_target - agent.me.location
                car_to_offset_perp = car_to_offset_target.cross(Vector(z=side_of_shot))  # perpendicular

                # The adjustment causes the car to circle around the dodge point in an effort to line up with the shot vector
                # The adjustment slowly decreases to 0 as the bot nears the time to jump
                adjustment = car_to_offset_target.angle2D(self.shot_vector) * T * 500  # size of adjustment
                # we don't adjust the final target if we are already jumping
                final_target += car_to_offset_perp.normalize() * adjustment

            distance_remaining = final_target.flat_dist(agent.me.location) - agent.me.hitbox.length / 3
            speed_required = distance_remaining / time_remaining
            agent.dbg_2d(f"Speed required: {speed_required}")

            if speed_required < 1900 and agent.me.boost > 50 and T > 2 and distance_remaining > 2560:
                agent.dbg_2d("Building speed")
                speed_required *= 0.75

        # Some adjustment to the final target to ensure it's inside the field and we don't try to drive through any goalposts or walls to reach it (again)
        final_target = cap_in_field(agent, final_target)
        local_final_target = agent.me.local_location(final_target)

        # the angle to the final target, in radians
        angle_to_target = abs(Vector(x=1).angle2D(local_final_target))
        # whether we should go forwards or backwards
        direction = 1 if angle_to_target < 1.6 or agent.me.local_velocity().x > 1000 else -1

        # drawing debug lines to show the dodge point and final target (which differs due to the adjustment)
        agent.line(agent.me.location, self.offset_target, agent.renderer.white())
        agent.line(self.offset_target-Vector(z=100), self.offset_target+Vector(z=100), agent.renderer.green())
        agent.line(final_target-Vector(z=100), final_target+Vector(z=100), agent.renderer.purple())

        velocity = defaultDrive(agent, speed_required * direction, local_final_target)[1]
        if velocity == 0: velocity = 1

        local_offset_target = agent.me.local_location(self.offset_target.flatten())
        time = distance_remaining / velocity

        vf = agent.me.velocity + agent.gravity * T
        local_vf = agent.me.local(vf.flatten())

        if T < 0.35 and T > 0.1 and min(abs(velocity) + dodge_impulse(agent), 2300) <= (abs(speed_required) if agent.me.boost > 12 else 1900) and ((abs(velocity) < 100 and distance_remaining < agent.me.hitbox.length / 2) or (abs(local_offset_target.y) < 92 and direction * local_vf.x >= direction * (local_offset_target.x - agent.me.hitbox.length / 3) and direction * local_offset_target.x > 0)):
            agent.push(flip(agent.me.local_location(self.offset_target), cancel=angle_to_target > 2))
        elif agent.me.airborne:
            agent.push(recovery(local_final_target if T > 0.5 else None))
        elif T <= 0 or (T > 0.35 and distance_remaining > 50 and not virxrlcu.ground_shot_is_viable(T, agent.boost_accel, agent.me.get_raw(agent), self.offset_target.z, tuple((final_target - agent.me.location).normalize()), distance_remaining)):
            # If we're out of time or not fast enough, we pop
            agent.pop()
            agent.shooting = False
            agent.shot_weight = -1
            agent.shot_time = -1
            if agent.me.airborne:
                agent.push(recovery())
        elif agent.boost_amount != 'unlimited' and agent.me.boost < 36 and angle_to_target < 0.03 and velocity > 500 and time > 1 and velocity + dodge_impulse(agent) <= speed_required:
            speed_gain_routine = wave_dash if agent.gravity.z < -450 and time < 3 and time > 1 else flip
            agent.push(speed_gain_routine(agent.me.local_location(self.offset_target)))
        elif agent.boost_amount != 'unlimited' and direction == -1 and velocity < 200 and distance_remaining / abs(velocity) > 4:
            agent.push(flip(agent.me.local_location(self.offset_target), True))


class generic_kickoff:
    def __init__(self):
        self.start_time = -1
        self.flip = False

    def run(self, agent: VirxERLU):
        if self.start_time == -1:
            self.start_time = agent.time

        if self.flip or agent.time - self.start_time > 3:
            agent.kickoff_done = True
            agent.pop()
            return

        target = agent.ball.location + Vector(y=(200 if agent.gravity.z < -600 and agent.gravity.z > -700 else 50)*side(agent.team))
        local_target = agent.me.local_location(target)

        defaultPD(agent, local_target)
        agent.controller.throttle = 1
        agent.controller.boost = True

        distance = local_target.magnitude()

        if distance < 550:
            self.flip = True
            agent.push(flip(agent.me.local_location(agent.foe_goal.location)))


class corner_kickoff:
    def __init__(self, direction):
        self.direction = direction
        self.start_time = None
        self.drive_right = None
        self.last_boost = 34
        self.flip = None
        self.flip_done = None

    def run(self, agent: VirxERLU):
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
                    self.flip = flip(Vector(70, -30 * sign(agent.me.location.x * side(agent.team))))

                self.flip_done = self.flip.run(agent, manual=True)
            else:
                agent.kickoff_done = True
                agent.pop()
                return

        self.last_boost = agent.me.boost

        if agent.time - self.start_time > 3.5:
            agent.kickoff_done = True
            agent.pop()


class corner_kickoff_boost:
    def __init__(self):
        self.goto = False

    def run(self, agent: VirxERLU):
        if not self.goto:
            self.goto = True
            agent.push(goto_boost(min(tuple(boost for boost in agent.boosts if boost.large and boost.active), key=lambda boost: boost.location.flat_dist(agent.me.location))))
        else:
            agent.pop()
            agent.kickoff_done = True


class back_offset_kickoff_boost:
    def __init__(self):
        self.small = False
        self.large = False
        self.small_boost = None
        self.large_boost = None
        self.goto = None

    def run(self, agent: VirxERLU):
        if not self.small:
            if self.goto is None:
                self.small_boost = min(tuple(boost for boost in agent.boosts if not boost.large and boost.active and abs(boost.location.y) > 3000 and abs(boost.location.x) > 50), key=lambda boost: boost.location.flat_dist(agent.me.location))
                self.large_boost = min(tuple(boost for boost in agent.boosts if boost.large and boost.active and abs(boost.location.y) > 3000), key=lambda boost: boost.location.flat_dist(agent.me.location))
                self.goto = goto(self.small_boost.location, self.large_boost.location)

            self.small = not self.small_boost.active
            self.goto.run(agent, manual=True)
            agent.controller.boost = True
        elif not self.large:
            agent.push(goto_boost(self.large_boost))
            self.large = True
        else:
            agent.pop()
            agent.kickoff_done = True


class back_offset_kickoff:
    def __init__(self):
        self.start_time = None
        self.do_boost = True
        self.boost_pad = None
        self.flip = None
        self.flip_done = None
        self.flip2 = None
        self.flip_done2 = None

    def run(self, agent: VirxERLU):
        if self.start_time is None:
            self.start_time = agent.time
            self.boost_pad = min(tuple(boost for boost in agent.boosts if not boost.large and boost.active and boost.location.x == 0 and abs(boost.location.y) < 3000), key=lambda boost: boost.location.flat_dist(agent.me.location))

        agent.controller.throttle = 1

        if self.do_boost:
            self.do_boost = agent.me.boost != 0
            agent.controller.boost = True
    
        if self.flip is None and agent.me.location.flat_dist(self.boost_pad.location + Vector(y=320 * side(agent.team))) > 480:
            defaultPD(agent, agent.me.local_location(self.boost_pad.location + Vector(y=320 * side(agent.team))))
        elif self.flip_done is None:
            if self.flip is None:
                self.boost_pad =  min(tuple(boost for boost in agent.boosts if not boost.large and boost.active and boost.location.x == 0 and abs(boost.location.y) < 2000), key=lambda boost: boost.location.flat_dist(agent.me.location))
                self.flip = flip(Vector(28.5, 71.5 * sign(agent.me.location.x * side(agent.team))))

            self.flip_done = self.flip.run(agent, manual=True)
        elif self.flip_done2 is None:
            if self.flip2 is None:
                self.flip2 = flip(agent.me.local_location(agent.ball.location))

            self.flip_done2 = self.flip2.run(agent, manual=True)
        else:
            agent.kickoff_done = True
            agent.pop()
            return

        if agent.time - self.start_time > 4.5:
            agent.kickoff_done = True
            agent.pop()


class recovery:
    # Point towards our velocity vector and land upright, unless we aren't moving very fast
    # A vector can be provided to control where the car points when it lands
    def __init__(self, target=None):
        self.target = target

    def run(self, agent: VirxERLU):
        target = agent.me.velocity.normalize() if self.target is None else (self.target - agent.me.location).normalize()

        landing_plane = virxrlcu.find_landing_plane(tuple(agent.me.location), tuple(agent.me.velocity), agent.gravity.z)

        d_switch = [
            "side wall",
            "side wall",
            "back wall",
            "back wall",
            "ceiling",
            "floor"
        ]

        agent.dbg_2d(f"Recovering towards the {d_switch[landing_plane]}")

        t_switch = [
            Vector(y=target.y, z=-1),
            Vector(y=target.y, z=-1),
            Vector(x=target.x, z=-1),
            Vector(x=target.x, z=-1),
            Vector(x=target.x, y=target.y),
            Vector(x=target.x, y=target.y)
        ]

        r_switch = [
            Vector(x=-1),
            Vector(x=1),
            Vector(y=-1),
            Vector(y=1),
            Vector(z=-1),
            Vector(z=1)
        ]

        defaultPD(agent, agent.me.local(t_switch[landing_plane]), up=agent.me.local(r_switch[landing_plane]))
        agent.controller.throttle = 1
        if not agent.me.airborne:
            agent.pop()


class ball_recovery:
    def __init__(self):
        self.recovery = recovery()

    def run(self, agent: VirxERLU):
        self.recovery.target = agent.ball.location
        self.recovery.target.y = cap(self.recovery.target.y, -5100, 5100)
        self.recovery.run(agent)


class short_shot:
    # This routine drives towards the ball and attempts to hit it towards a given target
    # It does not require ball prediction and kinda guesses at where the ball will be on its own
    def __init__(self, target):
        self.target = target
        self.start_time = None

    def run(self, agent: VirxERLU):
        if not agent.shooting or agent.shot_weight != -1:
            agent.shooting = True
            agent.shot_weight = -1

        if self.start_time is None:
            self.start_time = agent.time

        car_to_ball, distance = (agent.ball.location - agent.me.location).normalize(True)
        ball_to_target = (self.target - agent.ball.location).normalize()

        relative_velocity = car_to_ball.dot(agent.me.velocity-agent.ball.velocity)
        eta = cap(distance / cap(relative_velocity, 400, 2150), 0, 1.5) if relative_velocity != 0 else 1.5

        # If we are approaching the ball from the wrong side the car will try to only hit the very edge of the ball
        left_vector = car_to_ball.cross(Vector(z=1))
        right_vector = car_to_ball.cross(Vector(z=-1))
        target_vector = -ball_to_target.clamp2D(left_vector, right_vector)
        final_target = agent.ball.location + (target_vector*(distance/2))
        angle_to_target = abs(Vector(x=1).angle2D(agent.me.local_location(final_target)))
        distance_remaining = agent.me.location.dist(final_target) - agent.me.hitbox.length / 3

        # Some adjustment to the final target to ensure we don't try to drive through any goalposts to reach it
        final_target = cap_in_field(agent, final_target)
        local_final_target = agent.me.local_location(final_target)

        agent.line(final_target-Vector(z=100), final_target + Vector(z=100), (255, 255, 255))
        angles, velocity = defaultDrive(agent, 1410, local_final_target)

        if velocity == 0: velocity = 1

        if abs(angles[1]) < 0.05 and (eta < 0.45 or distance < 150):
            agent.pop()
            agent.shooting = False
            agent.shot_weight = -1
            agent.shot_time = -1
            agent.push(flip(agent.me.local(car_to_ball)))
        elif agent.boost_amount != 'unlimited' and angle_to_target < 0.03 and velocity > 600 and velocity < 2150 and distance_remaining / velocity > 3 and agent.me.location.z < 50:
            if agent.gravity.z < -450 and distance_remaining / velocity < 5:
                agent.push(wave_dash())
            else:
                agent.push(flip(local_final_target))
        elif angle_to_target < 2.2 and velocity < 200 and distance_remaining / abs(velocity) > 2 and agent.me.location.z < 50:
            agent.push(flip(local_final_target, True))
        elif agent.me.airborne:
            agent.push(recovery(local_final_target))


class ceiling_shot:
    def __init__(self):
        self.wait_target = Vector()
        self.target_location = None
        # Routine states
        self.collect_boost = True
        self.wait = False
        self.go_high = False
        self.falling = False

    def run(self, agent: VirxERLU):
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
            if agent.me.location.dist(self.wait_target) > 150:
                local_target = agent.me.local_location(self.wait_target)
                angle_to_target = abs(Vector(x=1).angle2D(local_target))
                direction = -1 if agent.me.location.z > 100 and angle_to_target >= 2.2 else 1

                defaultDrive(agent, 1400 * direction, local_target)
                agent.controller.boost = False
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
                local_target = agent.me.local_location(self.target_location)
                angle_to_target = abs(Vector(x=1).angle2D(local_target))

                defaultDrive(agent, 2300 if self.target_location.dist(agent.me.location) > 2560 else 1400, local_target)
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


class boost_down:
    def __init__(self):
        self.face = ball_recovery()

    def run(self, agent: VirxERLU):
        if agent.me.boost == 0:
            agent.pop()
            agent.push(self.face)

        target = agent.me.local(agent.me.forward.flatten()*100 - Vector(z=100))
        defaultPD(agent, target)
        if not agent.me.airborne:
            agent.pop()
        elif abs(Vector(x=1).angle(target)) < 0.5:
            agent.controller.boost = True
