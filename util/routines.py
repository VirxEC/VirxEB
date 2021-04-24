import random

import virxrlcu

from util.agent import (BallState, CarState, GameState, Physics, Vector3,
                        VirxERLU)
from util.utils import *

max_speed = 2300
throttle_accel = 66 + (2/3)
brake_accel = Vector(x=-3500)
boost_per_second = 33 + (1/3)
jump_max_duration = 0.2
jump_speed = 291 + (2/3)
jump_acc = 1458 + (1/3)
no_adjust_radians = 0.001
min_adjust_radians = 0.5
dodge_offset = 0.12


class wave_dash:
    def __init__(self, target=None):
        self.step = -1
        # 0 = forward, 1 = right, 2 = backwards, 3 = left
        self.direction = 0
        self.start_time = -1
        self.target = target

        if self.target is not None:
            self.direction = 0 if abs(self.target.x) > abs(self.target.y) else 1

            if (self.direction == 0 and self.target.x < 0) or (self.direction == 1 and self.target.y < 0):
                self.direction += 2

    def run(self, agent: VirxERLU):
        if self.start_time == -1:
            self.start_time = agent.time

        T = agent.time - self.start_time

        self.step += 1

        forward_target = agent.me.velocity.flatten().normalize() * (agent.me.hitbox.length / 2)

        target_switch = {
            0: forward_target + Vector(z=25),
            1: forward_target,
            2: forward_target - Vector(z=25),
            3: forward_target
        }

        target_up = {
            0: Vector(z=1),
            1: Vector(y=-1, z=1),
            2: Vector(z=1),
            3: Vector(y=1, z=1)
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
            agent.push(recovery())
        elif agent.me.location.z + (agent.me.velocity.z * 0.15) < 5:
            agent.controller.jump = True
            agent.controller.yaw = 0
            if self.direction in {0, 2}:
                agent.controller.roll = 0
                agent.controller.pitch = -1 if self.direction is 0 else 1
            else:
                agent.controller.roll = 1 if self.direction is 1 else -1
                agent.controller.pitch = 0


class double_jump:
    # Hits a target point at a target time towards a target direction
    def __init__(self, intercept_time, targets=None, weight=0):
        self.ball_location = None
        self.shot_vector = None
        self.offset_target = None
        self.intercept_time = intercept_time
        self.targets = targets
        self.weight = weight
        # Flags for what part of the routine we are in
        self.jumping = False
        self.dodged = False
        self.jump_time = -1
        self.needed_jump_time = -1
        self.counter = 0

        self.upgrade_intercept_time = None
        self.upgrade_targets = None
        self.upgrade_weight = 0
        self.upgrade = False

    def update(self, shot):
        self.upgrade_intercept_time = shot.intercept_time
        self.upgrade_targets = shot.targets
        self.upgrade_weight = shot.weight
        self.upgrade = True

    def preprocess(self, agent: VirxERLU):
        T = self.intercept_time - agent.time
        slice_n = math.ceil(T * 60) - 1
        ball = agent.ball_prediction_struct.slices[slice_n].physics.location
        ball_location = Vector(ball.x, ball.y, ball.z)
        car_to_ball_norm = (ball_location - agent.me.location).normalize()
        shot_vector = car_to_ball_norm if self.targets is None else car_to_ball_norm.clamp((self.targets[0] - ball_location).normalize(), (self.targets[1] - ball_location).normalize())

        if self.shot_vector is None or self.ball_location.dist(ball_location) > 5:
            self.ball_location = ball_location
            self.shot_vector = shot_vector
            self.offset_target = self.ball_location - (self.shot_vector * agent.ball_radius)
            self.needed_jump_time = round(virxrlcu.get_double_jump_time(round(self.offset_target.z - agent.me.location.z), round(agent.me.velocity.z), agent.gravity.z), 3)

    def run(self, agent: VirxERLU):
        # This routine is the same as jump_shot, but it's designed to hit the ball above 300uus and below 450uus without requiring boost
        if not agent.shooting:
            agent.shooting = True

        T = self.intercept_time - agent.time

        if (not self.jumping and T > 1.5 and agent.odd_tick % 2 == 0) or self.ball_location is None:
            if self.upgrade:
                self.intercept_time = self.upgrade_intercept_time
                self.targets = self.upgrade_targets
                self.weight = self.upgrade_weight
                self.upgrade = False

                T = self.intercept_time - agent.time

            self.preprocess(agent)

        agent.sphere(self.ball_location, agent.ball_radius)
        agent.dbg_2d(f"Needed jump time: {self.needed_jump_time}")

        car_to_ball = self.ball_location - agent.me.location
        final_target = self.offset_target.flatten()
        Tj = T - (self.needed_jump_time + dodge_offset)
        distance_remaining = None

        if Tj > 0 and self.targets is not None:
            angle_to_shot_vector = abs(car_to_ball.angle2D(self.shot_vector))
            if angle_to_shot_vector > no_adjust_radians:
                # whether we are to the left or right of the shot vector
                side_of_shot = sign(self.shot_vector.cross(Vector(z=1)).dot(car_to_ball))
                car_to_offset_target = final_target - agent.me.location
                car_to_offset_perp = car_to_offset_target.cross(Vector(z=side_of_shot)).normalize()  # perpendicular ray
                final_target += (-(self.shot_vector * (2560 - agent.ball_radius))) if angle_to_shot_vector > min_adjust_radians else (car_to_offset_perp * agent.me.hitbox.width * 0.5)

                if angle_to_shot_vector > min_adjust_radians:
                    ray_direction = (-self.shot_vector).rotate2D(side_of_shot * -min_adjust_radians)
                    distance_from_turn = ray_intersects_with_line(self.ball_location, ray_direction, agent.me.location, final_target)
                    true_final_target = self.offset_target + (car_to_offset_perp * agent.me.hitbox.width * 0.5)

                    turn_rad = turn_radius(abs(agent.me.local_velocity().x)) * 1.05
                    right = turn_rad * agent.me.right
                    if ray_intersects_with_circle(self.ball_location, ray_direction, agent.me.location + right, turn_rad) or ray_intersects_with_circle(self.ball_location, ray_direction, agent.me.location - right, turn_rad):
                        final_target = true_final_target

                    if distance_from_turn is not None:
                        car_turn_point = self.ball_location + ray_direction * distance_from_turn
                        part_dist = agent.me.location.flat_dist(car_turn_point) - turn_rad
                        distance_remaining = part_dist + car_turn_point.flat_dist(true_final_target)

        if distance_remaining is None:
            distance_remaining = final_target.flat_dist(agent.me.location)
            part_dist = distance_remaining

        # Some adjustment to the final target to ensure it's inside the field and we don't try to drive through any goalposts or walls to reach it
        final_target = cap_in_field(agent, final_target)
        local_final_target = agent.me.local_location(final_target)

        # whether we should go forwards or backwards
        angle_to_target = abs(Vector(x=1).angle2D(local_final_target))

        # drawing debug lines to show the dodge point and final target (which differs due to the adjustment)
        agent.line(agent.me.location, self.offset_target, agent.renderer.white())
        agent.line(self.offset_target-Vector(z=agent.ball_radius), self.offset_target+Vector(z=agent.ball_radius), agent.renderer.green())
        agent.line(final_target-Vector(z=agent.ball_radius), final_target+Vector(z=agent.ball_radius), agent.renderer.purple())

        vf = agent.me.velocity + agent.gravity * T

        distance_remaining = agent.me.local_location(self.offset_target).x if agent.me.airborne else distance_remaining
        distance_remaining -= agent.me.hitbox.length * 0.45
        distance_remaining = max(distance_remaining, 0)
        speed_required = distance_remaining / max(T, agent.delta_time)
        direction = 1 if angle_to_target < 1.6 or speed_required > 1410 else -1
        agent.dbg_2d(f"Speed required: {round(speed_required, 2)}")

        if not self.jumping:
            velocity = defaultDrive(agent, speed_required * direction, local_final_target)[1]
            if velocity == 0: velocity = 1

            local_offset_target = agent.me.local_location(self.offset_target).flatten()
            true_angle_to_target = abs(Vector(x=1).angle2D(local_offset_target))
            local_vf = agent.me.local(agent.me.velocity * T).flatten()
            dodge_time = part_dist / (abs(velocity) + dodge_impulse(agent)) - (self.needed_jump_time + dodge_offset)

            if (abs(velocity) < 100 and distance_remaining < agent.me.hitbox.length / 2 + agent.ball_radius) or (abs(local_offset_target.y) < agent.me.hitbox.width / 2 and direction * local_vf.x >= (direction * local_offset_target.x) - agent.me.hitbox.length * 0.6 and direction * local_offset_target.x > 0):
                self.jumping = T <= self.needed_jump_time + dodge_offset + 0.05
            elif agent.me.airborne:
                agent.push(recovery(local_final_target if Tj > 0 else None))
            elif T < self.needed_jump_time - agent.delta_time * 6 or (Tj > 0.05 and distance_remaining > agent.me.hitbox.length * 0.6 and not virxrlcu.double_jump_shot_is_viable(T, agent.boost_accel, tuple(agent.gravity), agent.me.get_raw(agent), self.offset_target.z, tuple((final_target - agent.me.location).flatten().normalize()), distance_remaining)):
                # If we're out of time or the ball was hit away or we just can't get enough speed, pop
                agent.pop()
                agent.shooting = False
                if agent.me.airborne:
                    agent.push(ball_recovery())
            elif dodge_time >= 1.2 and agent.time - agent.me.land_time > 0.5:
                if agent.me.boost < 48 and angle_to_target < 0.03 and (true_angle_to_target < 0.1 or distance_remaining > 4480) and velocity > 600 and velocity < speed_required - 50:
                    agent.push(flip(agent.me.local_location(self.offset_target)))
                elif direction == -1 and velocity < 200:
                    agent.push(flip(agent.me.local_location(self.offset_target), True))
        else:
            # Mark the time we started jumping so we know when to dodge
            if self.jump_time == -1:
                self.jump_time = agent.time

            jump_elapsed = agent.time - self.jump_time
            tau = jump_max_duration - jump_elapsed

            Tj2 = max(T - dodge_offset, agent.delta_time)

            xf = agent.me.location + agent.me.velocity * Tj2 + 0.5 * agent.gravity * Tj2 * Tj2

            if jump_elapsed == 0:
                vf += agent.me.up * jump_speed
                xf += agent.me.up * jump_speed * Tj2

            hf = vf
            vf += agent.me.up * jump_acc * tau
            xf += agent.me.up * jump_acc * tau * (Tj2 - 0.5 * tau)

            hf += agent.me.up * jump_speed
            vf += agent.me.up * jump_speed
            xf += agent.me.up * jump_speed * (Tj2 - tau)

            delta_x = self.offset_target - xf
            d_direction = delta_x.normalize()

            if T > 0 and direction == 1 and abs(agent.me.forward.dot(d_direction)) > 0.75:
                delta_v = delta_x.dot(agent.me.forward) / T
                if agent.me.boost > 0 and delta_v >= agent.boost_accel * 0.1:
                    agent.controller.boost = True
                    agent.controller.throttle = 1
                elif abs(delta_v) >= throttle_accel * agent.delta_time:
                    agent.controller.throttle = cap(delta_v / (throttle_accel * agent.delta_time), -1, 1)

            if T <= -0.4 or (not agent.me.airborne and self.counter == 4):
                agent.pop()
                agent.shooting = False
                agent.push(ball_recovery())
            elif jump_elapsed < jump_max_duration and hf.z <= self.offset_target.z:
                agent.controller.jump = True
            elif self.counter < 4:
                self.counter += 1

            if self.counter == 3:
                agent.controller.jump = True
            elif self.counter == 4:
                defaultPD(agent, agent.me.local_location(self.offset_target) * direction, upside_down=True)

            if self.counter < 3:
                defaultPD(agent, agent.me.local_location(self.offset_target.flatten()) * direction)

        l_vf = vf + agent.me.location
        agent.line(l_vf-Vector(z=agent.ball_radius), l_vf+Vector(z=agent.ball_radius), agent.renderer.red())


class Aerial:
    def __init__(self, intercept_time, targets=None, weight=0, fast_aerial=True):
        self.intercept_time = intercept_time
        self.fast_aerial = fast_aerial
        self.targets = targets
        self.weight = weight
        self.shot_vector = None
        self.offset_target = None
        self.ball_location = None
        self.jump_type_fast = None
        self.jumping = False
        self.dodging = False
        self.ceiling = False
        self.jump_time = -1
        self.counter = 0

        self.upgrade_intercept_time = None
        self.upgrade_fast_aerial = None
        self.upgrade_targets = None
        self.upgrade_weight = 0
        self.upgrade = False

    def update(self, shot):
        self.upgrade_intercept_time = shot.intercept_time
        self.upgrade_fast_aerial = shot.fast_aerial
        self.upgrade_targets = shot.targets
        self.upgrade_weight = shot.weight
        self.upgrade = True

    def preprocess(self, agent: VirxERLU):
        T = self.intercept_time - agent.time
        slice_n = math.ceil(T * 60) - 1
        ball = agent.ball_prediction_struct.slices[slice_n].physics.location
        ball_location = Vector(ball.x, ball.y, ball.z)
        car_to_ball_norm = (ball_location - agent.me.location).normalize()
        shot_vector = car_to_ball_norm if self.targets is None else car_to_ball_norm.clamp((self.targets[0] - ball_location).normalize(), (self.targets[1] - ball_location).normalize())

        if self.shot_vector is None or self.ball_location.dist(ball_location) > 5:
            self.ball_location = ball_location
            self.shot_vector = shot_vector
            self.offset_target = self.ball_location - (self.shot_vector * agent.ball_radius)

    def run(self, agent: VirxERLU):
        if not agent.shooting:
            agent.shooting = True

        T = self.intercept_time - agent.time

        if (T > 0.3 and agent.odd_tick % 2 == 0) or self.ball_location is None:
            if self.upgrade:
                self.intercept_time = self.upgrade_intercept_time
                self.fast_aerial = self.upgrade_fast_aerial
                self.targets = self.upgrade_targets
                self.weight = self.upgrade_weight
                self.upgrade = False

                T = self.intercept_time - agent.time

            self.preprocess(agent)

        final_target = self.offset_target
        if T > 0 and self.targets is not None:
            car_to_ball = self.ball_location - agent.me.location
            angle_to_shot_vector = abs(car_to_ball.angle2D(self.shot_vector))
            if angle_to_shot_vector > no_adjust_radians:
                # whether we are to the left or right of the shot vector
                side_of_shot = sign(self.shot_vector.cross(Vector(z=1)).dot(car_to_ball))
                car_to_offset_target = final_target - agent.me.location
                car_to_offset_perp = car_to_offset_target.cross(Vector(z=side_of_shot)).normalize()  # perpendicular ray
                final_target += car_to_offset_perp * agent.me.hitbox.width * 0.5

        agent.sphere(self.ball_location, agent.ball_radius)
        xf = agent.me.location + agent.me.velocity * T + 0.5 * agent.gravity * T * T
        vf = agent.me.velocity + agent.gravity * T

        if self.jumping or (self.jump_time == -1 and not agent.me.airborne):
            agent.dbg_2d("Jumping")

            if self.jump_time == -1:
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
                else:
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

        delta_x = final_target - xf
        direction = delta_x.normalize() if not self.jumping or not self.jump_type_fast else delta_x.flatten().normalize()

        agent.line(agent.me.location, agent.me.location + (direction * 250), agent.renderer.black())
        c_vf = vf + agent.me.location
        agent.line(c_vf - Vector(z=agent.ball_radius), c_vf + Vector(z=agent.ball_radius), agent.renderer.blue())
        agent.line(xf - Vector(z=agent.ball_radius), xf + Vector(z=agent.ball_radius), agent.renderer.red())
        agent.line(self.offset_target - Vector(z=agent.ball_radius), self.offset_target + Vector(z=agent.ball_radius), agent.renderer.green())

        delta_v = delta_x.dot(agent.me.forward) / T

        if self.counter in {0, 4}:
            target = agent.me.local(delta_x) if (delta_v >= agent.boost_accel * 0.1 + throttle_accel * agent.delta_time) or (T > 1 and delta_v >= throttle_accel * agent.delta_time * 0.1) else agent.me.local_location(self.offset_target)

            if self.jumping and self.jump_type_fast:
                defaultPD(agent, target)
            elif virxrlcu.find_landing_plane(tuple(agent.me.location), tuple(agent.me.velocity), agent.gravity.z) == 4:
                defaultPD(agent, target, upside_down=True)
            else:
                defaultPD(agent, target, upside_down=agent.me.location.z > self.offset_target.z)

        # only boost/throttle if we're facing the right direction
        if T > 0 and abs(agent.me.forward.angle(direction)) < 0.5 and not self.jumping:
            if T > 0.3: agent.controller.roll = 1 if self.shot_vector.z < 0 else -1
            # the change in velocity the bot needs to put it on an intercept course with the target
            if agent.me.airborne and agent.me.boost > 0 and delta_v >= agent.boost_accel * 0.1 + throttle_accel * agent.delta_time:
                agent.controller.boost = True
                agent.controller.throttle = 1
            elif abs(delta_v) >= throttle_accel * agent.delta_time * 0.1:
                agent.controller.throttle = cap(delta_v / (throttle_accel * agent.delta_time), -1, 1)

        if T <= -0.2 or (not self.jumping and not agent.me.airborne) or (not self.jumping and ((delta_v >= agent.boost_accel * 0.1 + throttle_accel * agent.delta_time) or (T > 1 and delta_v >= throttle_accel * agent.delta_time * 0.1)) and not virxrlcu.aerial_shot_is_viable(T, agent.boost_accel, tuple(agent.gravity), agent.me.get_raw(agent), tuple(self.offset_target))):
            agent.pop()
            agent.shooting = False
            agent.push(ball_recovery())
        elif not self.ceiling and not agent.me.doublejumped and T < 0.1:
            agent.dbg_2d("Flipping")
            vector = agent.me.local_location(self.offset_target).flatten().normalize()
            target_angle = math.atan2(vector.y, vector.x)
            agent.controller.pitch = -math.cos(target_angle)
            agent.controller.yaw = math.sin(target_angle)
            agent.controller.throttle = -1 if agent.controller.pitch > 0 else 1
            agent.controller.jump = True


class flip:
    # Flip takes a vector in local coordinates and flips/dodges in that direction
    # cancel causes the flip to cancel halfway through, which can be used to half-flip
    def __init__(self, vector, cancel=False):
        target_angle = math.atan2(vector.y, vector.x)
        self.yaw = math.sin(target_angle)
        self.pitch = -math.cos(target_angle)
        self.throttle = -1 if self.pitch > 0 else 1

        self.cancel = cancel
        # the time the jump began
        self.time = -1
        # keeps track of the frames the jump button has been released
        self.counter = 0

    def run(self, agent: VirxERLU, manual=False, recovery_target=None):
        if self.time == -1:
            self.time = agent.time

        elapsed = agent.time - self.time
        agent.controller.throttle = self.throttle

        if elapsed < 0.1:
            agent.controller.jump = True
        elif elapsed >= 0.1 and self.counter < 3:
            agent.controller.pitch = self.pitch
            agent.controller.yaw = self.yaw
            agent.controller.jump = False
            self.counter += 1
        elif agent.me.airborne and (elapsed < 0.4 or (not self.cancel and elapsed < 0.9)):
            agent.controller.pitch = self.pitch
            agent.controller.yaw = self.yaw
            agent.controller.jump = True
        else:
            if not manual:
                agent.pop()
            agent.push(recovery(recovery_target))
            return True


class brake:
    @staticmethod
    def run(agent: VirxERLU, manual=False):
        if agent.cheating and agent.me.velocity.flatten().magnitude() > 1410:
            new_velocity = agent.me.velocity.flatten().scale(1400)
            cars = { agent.index: CarState(Physics(velocity=Vector3(new_velocity.x, new_velocity.y))) }
            agent.set_game_state(GameState(cars=cars))

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
    # Brake brings the car to slow down to 0 when it gets to it's destination
    # Slow is for small targets, and it forces the car to slow down a bit when it gets close to the target
    def __init__(self, target, vector=None, brake=False, slow=False):
        self.target = target
        self.vector = vector
        self.brake = brake
        self.slow = slow

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

        if not self.brake and not manual and distance_remaining < 320:
            agent.pop()
            return

        final_target = self.target.copy().flatten()

        if self.vector is not None:
            # See comments for adjustment in jump_shot for explanation
            side_of_vector = sign(self.vector.cross(Vector(z=1)).dot(car_to_target))
            car_to_target_perp = car_to_target.cross(Vector(z=side_of_vector)).normalize()
            adjustment = car_to_target.angle2D(self.vector) * distance_remaining / 3.14
            final_target += car_to_target_perp * adjustment

        final_target = cap_in_field(agent, final_target)  # Some adjustment to the final target to ensure it's inside the field and we don't try to drive through any goalposts to reach it
        local_target = agent.me.local_location(final_target)
        angle_to_target = abs(Vector(x=1).angle2D(local_target))
        true_angle_to_target = abs(Vector(x=1).angle2D(agent.me.local_location(self.target)))
        direction = 1 if angle_to_target < 1.6 or agent.me.local_velocity().x > 1000 else -1
        agent.dbg_2d(f"Angle to target: {round(angle_to_target, 1)}")

        velocity = defaultDrive(agent, (2300 if distance_remaining > 1280 or not self.slow else cap(distance_remaining * 2, 1200, 2300)) * direction, local_target)[1]
        if distance_remaining < 2560: agent.controller.boost = False

        # this is to break rule 1's with TM8'S ONLY
        # 251 is the distance between center of the 2 longest cars in the game, with a bit extra
        if agent.num_friends > 0 and agent.me.local_velocity().x < 50 and agent.controller.throttle == 1 and min(agent.me.location.flat_dist(car.location) for car in agent.alive_friends) < 251:
            if self.rule1_timer == -1:
                self.rule1_timer = agent.time
            elif agent.time - self.rule1_timer > 1.5:
                agent.push(flip(Vector(y=250)))
                return
        elif self.rule1_timer != -1:
            self.rule1_timer = -1

        dodge_time = distance_remaining / (abs(velocity) + dodge_impulse(agent)) - 0.8

        if agent.me.airborne:
            agent.push(recovery(self.target))
        elif dodge_time >= 1.2 and agent.time - agent.me.land_time > 0.2:
            if agent.me.boost < 48 and angle_to_target < 0.03 and (true_angle_to_target < 0.1 or distance_remaining > 4480) and velocity > 600:
                agent.push(flip(agent.me.local_location(self.target)))
            elif direction == -1 and velocity < 200:
                agent.push(flip(agent.me.local_location(self.target), True))


class shadow:
    def __init__(self):
        self.goto = goto(Vector(), brake=True)
        self.retreat = retreat()

    def run(self, agent: VirxERLU):
        ball_loc = self.get_ball_loc(agent, True)
        target = self.get_target(agent, ball_loc)

        if self.switch_to_retreat(agent, ball_loc, target):
            agent.pop()
            agent.push(retreat())
            return

        self_to_target = agent.me.location.flat_dist(target)

        if self_to_target < 100 * (agent.me.velocity.magnitude() / 500) and ball_loc.y < -640 and agent.me.velocity.magnitude() < 50 and abs(Vector(x=1).angle2D(agent.me.local_location(agent.ball.location))) > 1:
            agent.pop()
            if agent.num_friends > 1:
                agent.push(face_target(ball=True))
        else:
            self.goto.target = target
            self.goto.vector = ball_loc * Vector(y=side(agent.team)) if target.y * side(agent.team) < 1280 else None
            self.goto.run(agent)

    @staticmethod
    def switch_to_retreat(agent, ball, target):
        return agent.me.location.y * side(agent.team) < ball.y or ball.y > 2560 or target.y * side(agent.team) > 4480 or agent.is_own_goal

    def is_viable(self, agent, ignore_distance=False):
        ball_loc = self.get_ball_loc(agent)
        target = self.get_target(agent, ball_loc)

        return (ignore_distance or agent.me.location.flat_dist(target) > 320) and not self.switch_to_retreat(agent, ball_loc, target)

    def get_ball_loc(self, agent, render=False):
        ball_slice = agent.ball.location if agent.me.minimum_time_to_ball == 7 and agent.enemy_time_to_ball == 7 else agent.ball_prediction_struct.slices[min(round(agent.enemy_time_to_ball * 1.15 * 60), agent.ball_prediction_struct.num_slices - 1) if agent.me.minimum_time_to_ball == 7 else agent.min_intercept_slice].physics.location
        ball_loc = Vector(ball_slice.x, ball_slice.y)
        if render: agent.sphere(ball_loc + Vector(z=agent.ball_radius), agent.ball_radius, color=agent.renderer.black())
        ball_loc.y *= side(agent.team)

        if ball_loc.y < -2560 or (ball_loc.y < agent.ball.location.y * side(agent.team)):
            ball_loc = Vector(agent.ball.location.x, agent.ball.location.y * side(agent.team) - 640)

        return ball_loc

    def get_target(self, agent: VirxERLU, ball_loc=None):
        if ball_loc is None:
            ball_loc = self.get_ball_loc(agent)

        goal_diff = agent.game.friend_score - agent.game.foe_score

        if agent.num_friends > 0 and agent.playstyle is agent.playstyles.Neutral:
            distance = 3840
        elif agent.playstyle is agent.playstyles.Defensive:
            distances = [
                1920,
                3840,
                4800
            ]
            distance = distances[min(agent.num_friends, 2)]
        else:
            distance = 2560

        if agent.num_foes != 0 and (agent.playstyle is not agent.playstyles.Defensive or agent.num_friends == 1):
            # factor in both the enemy's time to ball and the goal differential
            distance *= cap(1 - (agent.enemy_time_to_ball / (8 - min(goal_diff, 6))), 0.5, 1) if goal_diff >= 0 else 1
            enemy_angle = abs(Vector(x=1).angle2D(agent.closest_foes[0].local_location(agent.ball.location)))
            if enemy_angle < 1:
                # factor in the enemy's speed - if they're travel at the ball at a high speed, then we need to back up because they're going to hit it hard
                distance *= min(agent.closest_foes[0].velocity.magnitude() - agent.me.velocity.magnitude() / 1000, 1 if agent.enemy_time_to_ball > 4 else 2)
            else:
                distance /= enemy_angle
            # make the minimum distance be 1920
            distance = max(distance, 1920)

        target = Vector(y=(ball_loc.y + distance) * side(agent.team))
        if target.y * side(agent.team) > -1280:
            # find the proper x coord for us to stop a shot going to the net
            # y = mx + b <- yes, finally! 7th grade math is paying off xD
            p1 = self.retreat.get_target(agent)
            p2 = ball_loc * Vector(x=1, y=side(agent.team))
            try:
                m = (p2.y - p1.y) / (p2.x - p1.x)
                b = p1.y - (m * p1.x)
                # x = (y - b) / m
                target.x = (target.y - b) / m
            except ZeroDivisionError:
                target.x = 0
        else:
            target.x = (abs(ball_loc.x) + 640) * sign(ball_loc.x)

        return Vector(target.x, target.y)


class retreat:
    def __init__(self):
        self.goto = goto(Vector(), brake=True)

    def run(self, agent: VirxERLU):
        ball = self.get_ball_loc(agent, render=True)
        target = self.get_target(agent, ball=ball)

        if shadow().is_viable(agent, ignore_distance=True):
            agent.pop()
            agent.push(shadow())
            return

        self_to_target = agent.me.location.flat_dist(target)

        if self_to_target < agent.me.hitbox.width:
            agent.pop()

            if agent.me.local_velocity().x > throttle_accel:
                agent.push(brake())
            return

        self.goto.target = target
        self.goto.run(agent)

    def is_viable(self, agent):
        return agent.me.location.flat_dist(self.get_target(agent)) > 320 and not shadow().is_viable(agent, ignore_distance=True)

    def get_ball_loc(self, agent: VirxERLU, render=False):
        if agent.is_own_goal:
            ball_slice = agent.own_goal["location"]
        else:
            ball_slice = agent.ball.location if agent.me.minimum_time_to_ball == 7 and agent.enemy_time_to_ball == 7 else agent.ball_prediction_struct.slices[min(round(agent.enemy_time_to_ball * 1.15 * 60), agent.ball_prediction_struct.num_slices - 1) if agent.me.minimum_time_to_ball == 7 else agent.min_intercept_slice].physics.location

        ball = Vector(ball_slice.x, ball_slice.y)
        if render: agent.sphere(ball + Vector(z=agent.ball_radius), agent.ball_radius, color=agent.renderer.black())
        ball.y *= side(agent.team)

        if ball.y < agent.ball.location.y * side(agent.team):
            ball = Vector(agent.ball.location.x, agent.ball.location.y * side(agent.team) + 640)

        return ball

    @staticmethod
    def friend_near_target(agent: VirxERLU, target):
        for car in agent.friends:
            if car.location.dist(target) < 400:
                return True
        return False

    def get_target(self, agent: VirxERLU, ball=None):
        target = None
        if ball is None:
            ball = self.get_ball_loc(agent)
        self_team = side(agent.team)

        horizontal_offset = 150
        outside_goal_offset = -125
        inside_goal_offset = 150

        if agent.is_own_goal:
            target = Vector(ball.x, agent.friend_goal.location.y)

        elif agent.num_foes != 0 and (agent.num_friends == 0 or agent.me.minimum_time_to_ball < agent.friend_times[0].minimum_time_to_ball):
            real_ball = ball * Vector(y=self_team)
            s = (real_ball - agent.closest_foes[0].location).normalize()
            start = (agent.friend_goal.left_post - real_ball).normalize()
            end = (agent.friend_goal.right_post - real_ball).normalize()

            right = s.dot(end.cross((0, 0, -1))) < 0
            left = s.dot(start.cross((0, 0, -1))) > 0

            if (right and left) if end.dot(start.cross((0, 0, -1))) > 0 else (right or left):
                target = Vector(y=agent.friend_goal.location.y)
                p1 = real_ball
                p2 = real_ball - s
                try:
                    m = (p2.y - p1.y) / (p2.x - p1.x)
                    b = p1.y - (m * p1.x)
                    # x = (y - b) / m
                    target.x = (target.y - b) / m
                except ZeroDivisionError:
                    target.x = 0
            elif start.dot(s) < end.dot(s):
                target = agent.friend_goal.right_post
            else:
                target = agent.friend_goal.left_post
        else:
            if ball.y < -640:
                target = agent.friend_goal.location
            elif ball.x * self_team < agent.friend_goal.right_post.x * self_team:
                target = agent.friend_goal.right_post

                while self.friend_near_target(agent, target):
                    target.x = (target.x * self_team + horizontal_offset * self_team) * self_team
            elif ball.x * self_team > agent.friend_goal.left_post.x * self_team:
                target = agent.friend_goal.left_post

                while self.friend_near_target(agent, target):
                    target.x = (target.x * self_team - horizontal_offset * self_team) * self_team
            else:
                target = agent.friend_goal.location
                target.x = ball.x

                while self.friend_near_target(agent, target):
                    target.x = (target.x * self_team - horizontal_offset * sign(ball.x) * self_team) * self_team

        target = target.copy()
        target.y += (inside_goal_offset if abs(target.x) < 800 else outside_goal_offset) * side(agent.team)

        return target.flatten()


class face_target:
    def __init__(self, target=None, ball=False):
        self.target = target
        self.ball = ball
        self.start_loc = None
        self.counter = 0

    @staticmethod
    def get_ball_target(agent):
        ball = agent.ball.location if agent.me.minimum_time_to_ball == 7 else agent.ball_prediction_struct.slices[agent.min_intercept_slice].physics.location
        return Vector(ball.x, ball.y)

    def run(self, agent: VirxERLU):
        if self.ball:
            target = self.get_ball_target(agent) - agent.me.location
        else:
            target = agent.me.velocity if self.target is None else self.target - agent.me.location

        if agent.gravity.z < -550 and agent.gravity.z > -750:
            if self.counter == 0 and abs(Vector(x=1).angle(target)) <= 0.05:
                agent.pop()
                return

            if self.counter == 0 and agent.me.airborne:
                self.counter = 3

            if self.counter < 3:
                self.counter += 1

            target = agent.me.local(target.flatten())
            if self.counter < 3:
                agent.controller.jump = True
            elif agent.me.airborne and abs(Vector(x=1).angle(target)) > 0.05:
                defaultPD(agent, target)
            else:
                agent.pop()
        else:
            target = agent.me.local(target.flatten())
            angle_to_target = abs(Vector(x=1).angle(target))
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
        self.goto = goto(self.boost.location, slow=not self.boost.large)

    def run(self, agent: VirxERLU):
        if not self.boost.active or agent.me.boost == 100:
            agent.pop()
            return

        self.goto.run(agent, manual=True)


class jump_shot:
    # Hits a target point at a target time towards a target direction
    def __init__(self, intercept_time, targets=None, weight=0):
        self.ball_location = None
        self.shot_vector = None
        self.offset_target = None
        self.intercept_time = intercept_time
        self.targets = targets
        self.weight = weight
        # Flags for what part of the routine we are in
        self.jumping = False
        self.dodging = False
        self.counter = 0
        self.jump_time = -1
        self.needed_jump_time = -1

        self.upgrade_intercept_time = None
        self.upgrade_targets = None
        self.upgrade_weight = 0
        self.upgrade = False

    def update(self, shot):
        self.upgrade_intercept_time = shot.intercept_time
        self.upgrade_targets = shot.targets
        self.upgrade_weight = shot.weight
        self.upgrade = True

    def preprocess(self, agent: VirxERLU):
        T = self.intercept_time - agent.time
        slice_n = math.ceil(T * 60) - 1
        ball = agent.ball_prediction_struct.slices[slice_n].physics.location
        ball_location = Vector(ball.x, ball.y, ball.z)
        car_to_ball_norm = (ball_location - agent.me.location).normalize()
        shot_vector = car_to_ball_norm if self.targets is None else car_to_ball_norm.clamp((self.targets[0] - ball_location).normalize(), (self.targets[1] - ball_location).normalize())

        if self.shot_vector is None or self.ball_location.dist(ball_location) > 5:
            self.ball_location = ball_location
            self.shot_vector = shot_vector
            self.offset_target = self.ball_location - (self.shot_vector * agent.ball_radius)
            self.needed_jump_time = round(virxrlcu.get_jump_time(round(self.offset_target.z - agent.me.location.z), round(agent.me.velocity.z), agent.gravity.z), 3)

    def run(self, agent: VirxERLU):
        if not agent.shooting:
            agent.shooting = True

        T = self.intercept_time - agent.time

        if (not self.jumping and T > 1 and agent.odd_tick % 2 == 0) or self.ball_location is None:
            if self.upgrade:
                self.targets = self.upgrade_targets
                self.intercept_time = self.upgrade_intercept_time
                self.weight = self.upgrade_weight
                self.upgrade = False

                T = self.intercept_time - agent.time

            self.preprocess(agent)

        agent.sphere(self.ball_location, agent.ball_radius)
        agent.dbg_2d(f"Needed jump time: {self.needed_jump_time}")

        car_to_ball = self.ball_location - agent.me.location
        final_target = self.offset_target.flatten()
        distance_remaining = None
        Tj = T - (self.needed_jump_time + dodge_offset)

        if Tj > 0 and self.targets is not None:
            angle_to_shot_vector = abs(car_to_ball.angle2D(self.shot_vector))
            if angle_to_shot_vector > no_adjust_radians:
                # whether we are to the left or right of the shot vector
                side_of_shot = sign(self.shot_vector.cross(Vector(z=1)).dot(car_to_ball))
                car_to_offset_target = final_target - agent.me.location
                car_to_offset_perp = car_to_offset_target.cross(Vector(z=side_of_shot)).normalize()  # perpendicular ray
                final_target += (-(self.shot_vector * (2560 - agent.ball_radius))) if angle_to_shot_vector > min_adjust_radians else (car_to_offset_perp * agent.me.hitbox.width * 0.5)

                if angle_to_shot_vector > min_adjust_radians:
                    ray_direction = (-self.shot_vector).rotate2D(side_of_shot * -min_adjust_radians)
                    distance_from_turn = ray_intersects_with_line(self.ball_location, ray_direction, agent.me.location, final_target)
                    true_final_target = self.offset_target + (car_to_offset_perp * agent.me.hitbox.width * 0.5)

                    turn_rad = turn_radius(abs(agent.me.local_velocity().x)) * 1.05
                    right = turn_rad * agent.me.right
                    if ray_intersects_with_circle(self.ball_location, ray_direction, agent.me.location + right, turn_rad) or ray_intersects_with_circle(self.ball_location, ray_direction, agent.me.location - right, turn_rad):
                        final_target = true_final_target

                    if distance_from_turn is not None:
                        car_turn_point = self.ball_location + ray_direction * distance_from_turn
                        part_dist = agent.me.location.flat_dist(car_turn_point) - turn_rad
                        distance_remaining = part_dist + car_turn_point.flat_dist(true_final_target)

        if distance_remaining is None:
            distance_remaining = final_target.flat_dist(agent.me.location)
            part_dist = distance_remaining

        # Some adjustment to the final target to ensure it's inside the field and we don't try to drive through any goalposts or walls to reach it (again)
        final_target = cap_in_field(agent, final_target)
        local_final_target = agent.me.local_location(final_target)

        # whether we should go forwards or backwards
        angle_to_target = abs(Vector(x=1).angle2D(agent.me.local_location(agent.ball.location) if self.jumping else local_final_target))

        # drawing debug lines to show the dodge point and final target (which differs due to the adjustment)
        agent.line(agent.me.location, self.offset_target, agent.renderer.white())
        agent.line(self.offset_target-Vector(z=agent.ball_radius), self.offset_target+Vector(z=agent.ball_radius), agent.renderer.green())
        agent.line(final_target-Vector(z=agent.ball_radius), final_target+Vector(z=agent.ball_radius), agent.renderer.purple())

        vf = agent.me.velocity + agent.gravity * T

        distance_remaining -= agent.me.hitbox.length * 0.45
        distance_remaining = max(distance_remaining, 0)
        speed_required = distance_remaining / max(T, agent.delta_time)
        direction = 1 if angle_to_target < 1.6 or speed_required > 1410 else -1
        agent.dbg_2d(f"Speed required: {round(speed_required, 2)}")

        if not self.jumping:
            velocity = defaultDrive(agent, speed_required * direction, local_final_target)[1]
            if velocity == 0: velocity = 1

            local_vf = agent.me.local(agent.me.velocity * T).flatten()
            dodge_time = part_dist / (abs(velocity) + dodge_impulse(agent)) - (self.needed_jump_time + dodge_offset)

            if (abs(velocity) < 100 and distance_remaining < agent.me.hitbox.length) or (abs(local_final_target.y) < agent.me.hitbox.width and direction * local_vf.x >= (direction * local_final_target.x) - agent.me.hitbox.length * 0.6 and direction * local_final_target.x > 0):
                self.jumping = T <= self.needed_jump_time + dodge_offset + 0.05
            elif agent.me.airborne:
                agent.push(recovery(final_target if Tj > 0 else None))
            elif T < self.needed_jump_time - agent.delta_time * 6 or (Tj > 0.05 and distance_remaining > agent.me.hitbox.length / 2 and not virxrlcu.jump_shot_is_viable(T, agent.boost_accel, tuple(agent.gravity), agent.me.get_raw(agent), self.offset_target.z, tuple((final_target - agent.me.location).flatten().normalize()), distance_remaining)):
                # If we're out of time or not fast enough to be within 45 units of target at the intercept time, we pop
                agent.pop()
                agent.shooting = False
                if agent.me.airborne:
                    agent.push(recovery())
            elif dodge_time >= 1.2 and agent.time - agent.me.land_time > 0.5:
                if agent.me.boost < 48 and angle_to_target < 0.03 and velocity < speed_required - 50 and velocity - speed_required < dodge_impulse(agent) * 3:
                    agent.push(flip(agent.me.local_location(self.offset_target)))
                elif direction == -1 and velocity < 200:
                    agent.push(flip(agent.me.local_location(self.offset_target), True))
        else:
            if self.jump_time == -1:
                self.jump_time = agent.time

            jump_elapsed = agent.time - self.jump_time
            tau = jump_max_duration - jump_elapsed

            Tj2 = max(T - dodge_offset, agent.delta_time)

            xf = agent.me.location + agent.me.velocity * Tj2 + 0.5 * agent.gravity * Tj2 * Tj2

            if jump_elapsed == 0:
                vf += agent.me.up * jump_speed
                xf += agent.me.up * jump_speed * Tj2

            hf = vf.z
            vf += agent.me.up * jump_acc * tau
            xf += agent.me.up * jump_acc * tau * (Tj2 - 0.5 * tau)

            delta_x = self.offset_target - xf
            d_direction = delta_x.normalize()

            if T > 0 and abs(agent.me.forward.angle(d_direction)) < 0.5:
                delta_v = delta_x.dot(agent.me.forward) / T
                if agent.me.airborne and agent.me.boost > 0 and delta_v >= agent.boost_accel * 0.1 + throttle_accel * agent.delta_time:
                    agent.controller.boost = True
                    agent.controller.throttle = 1
                elif abs(delta_v) >= throttle_accel * agent.delta_time * 0.1:
                    agent.controller.throttle = cap(delta_v / (throttle_accel * agent.delta_time), -1, 1)

            if T <= -0.8 or (not agent.me.airborne and self.counter >= 3):
                agent.pop()
                agent.shooting = False
                agent.push(recovery())
                return
            else:
                if self.counter == 3 and T < dodge_offset:
                    # Get the required pitch and yaw to flip correctly
                    vector = Vector()
                    if agent.me.location.flat_dist(agent.ball.location) > agent.me.hitbox.width + agent.ball_radius:
                        vector = agent.me.local((agent.ball.location - (self.shot_vector * agent.ball_radius) - agent.me.location).flatten()).normalize()

                    if vector.magnitude() == 0:
                        vector = agent.me.local(self.shot_vector.flatten()).normalize()

                    # tnx Impossibum...

                    target_angle = math.atan2(vector.y, vector.x)
                    self.y = math.sin(target_angle)
                    self.p = -math.cos(target_angle)
                    self.t = -1 if self.p > 0 else 1

                    # dodge
                    agent.controller.throttle = self.t
                    agent.controller.pitch = self.p
                    agent.controller.yaw = self.y
                    agent.controller.jump = True

                    self.counter += 1
                elif self.counter > 3:
                    agent.controller.throttle = self.t
                    agent.controller.pitch = self.p
                    agent.controller.yaw = self.y
                    agent.controller.jump = True
                else:
                    # Face the target as much as possible
                    defaultPD(agent, agent.me.local_location(final_target + Vector(z=self.offset_target.z)) * direction)

                if jump_elapsed <= jump_max_duration and hf <= self.offset_target.z:
                    # Initial jump to get airborne + we hold the jump button for extra power as required
                    agent.controller.jump = True
                elif self.counter < 3:
                    # Make sure we aren't jumping for at least 3 frames
                    self.counter += 1

        l_vf = vf + agent.me.location
        agent.line(l_vf-Vector(z=agent.ball_radius), l_vf+Vector(z=agent.ball_radius), agent.renderer.red())


class ground_shot:
    # Hits a target point at a target time towards a target direction
    def __init__(self, intercept_time, targets=None, weight=0):
        self.ball_location = None
        self.shot_vector = None
        self.offset_target = None
        self.intercept_time = intercept_time
        self.targets = targets
        self.weight = weight

        self.upgrade_intercept_time = None
        self.upgrade_targets = None
        self.upgrade_weight = 0
        self.upgrade = False

    def update(self, shot):
        self.upgrade_intercept_time = shot.intercept_time
        self.upgrade_targets = shot.targets
        self.upgrade_weight = shot.weight
        self.upgrade = True

    def preprocess(self, agent: VirxERLU):
        T = self.intercept_time - agent.time
        slice_n = math.ceil(T * 60) - 1
        ball = agent.ball_prediction_struct.slices[slice_n].physics.location
        ball_location = Vector(ball.x, ball.y, ball.z)
        car_to_ball_norm = (ball_location - agent.me.location).normalize()
        shot_vector = car_to_ball_norm if self.targets is None else car_to_ball_norm.clamp((self.targets[0] - ball_location).normalize(), (self.targets[1] - ball_location).normalize())

        if self.shot_vector is None or self.ball_location.dist(ball_location) > 5:
            self.ball_location = ball_location
            self.shot_vector = shot_vector
            self.offset_target = self.ball_location - (self.shot_vector * agent.ball_radius)

    def run(self, agent: VirxERLU):
        if not agent.shooting:
            agent.shooting = True

        T = self.intercept_time - agent.time

        if (T > 0.2 and agent.odd_tick % 2 == 0) or self.ball_location is None:
            if self.upgrade:
                self.intercept_time = self.upgrade_intercept_time
                self.targets = self.upgrade_targets
                self.weight = self.upgrade_weight
                self.upgrade = False

                T = self.intercept_time - agent.time

            self.preprocess(agent)

        agent.sphere(self.ball_location, agent.ball_radius)
        # Capping T above 0 to prevent division problems
        time_remaining = cap(T, 0.000001, 6)

        car_to_ball = agent.ball.location - agent.me.location
        final_target = self.offset_target.flatten()
        distance_remaining = None

        if self.targets is not None:
            angle_to_shot_vector = abs(car_to_ball.angle2D(self.shot_vector))
            if angle_to_shot_vector > no_adjust_radians:
                # whether we are to the left or right of the shot vector
                side_of_shot = sign(self.shot_vector.cross(Vector(z=1)).dot(car_to_ball))
                car_to_offset_target = final_target - agent.me.location
                car_to_offset_perp = car_to_offset_target.cross(Vector(z=side_of_shot)).normalize()  # perpendicular ray
                final_target += (-(self.shot_vector * (2560 - agent.ball_radius))) if angle_to_shot_vector > min_adjust_radians else (car_to_offset_perp * agent.me.hitbox.width * 0.5)

                if angle_to_shot_vector > min_adjust_radians:
                    ray_direction = (-self.shot_vector).rotate2D(side_of_shot * -min_adjust_radians)
                    distance_from_turn = ray_intersects_with_line(self.ball_location, ray_direction, agent.me.location, final_target)
                    true_final_target = self.offset_target + (car_to_offset_perp * agent.me.hitbox.width * 0.5)

                    turn_rad = turn_radius(abs(agent.me.local_velocity().x)) * 1.05
                    right = turn_rad * agent.me.right
                    if ray_intersects_with_circle(self.ball_location, ray_direction, agent.me.location + right, turn_rad) or ray_intersects_with_circle(self.ball_location, ray_direction, agent.me.location - right, turn_rad):
                        final_target = true_final_target

                    if distance_from_turn is not None:
                        car_turn_point = self.ball_location + ray_direction * distance_from_turn
                        part_dist = agent.me.location.flat_dist(car_turn_point) - turn_rad
                        distance_remaining = part_dist + car_turn_point.flat_dist(true_final_target)

        if distance_remaining is None:
            distance_remaining = final_target.flat_dist(agent.me.location)
            part_dist = distance_remaining

        distance_remaining -= agent.me.hitbox.length * 0.45
        distance_remaining = max(distance_remaining, 0)

        speed_required = distance_remaining / time_remaining
        agent.dbg_2d(f"Speed required: {round(speed_required)}")

        # Some adjustment to the final target to ensure it's inside the field and we don't try to drive through any goalposts or walls to reach it (again)
        final_target = cap_in_field(agent, final_target)
        local_final_target = agent.me.local_location(final_target)

        # the angle to the final target, in radians
        angle_to_target = abs(Vector(x=1).angle2D(local_final_target))
        # whether we should go forwards or backwards
        direction = 1 if angle_to_target < 1.6 or speed_required > 1410 or (abs(speed_required) < 100 and angle_to_target < 1.7) else -1

        # drawing debug lines to show the dodge point and final target (which differs due to the adjustment)
        agent.line(agent.me.location, self.offset_target, agent.renderer.white())
        agent.line(self.offset_target-Vector(z=agent.ball_radius), self.offset_target+Vector(z=agent.ball_radius), agent.renderer.green())
        agent.line(final_target-Vector(z=agent.ball_radius), final_target+Vector(z=agent.ball_radius), agent.renderer.purple())

        velocity = defaultDrive(agent, speed_required * direction, local_final_target)[1]
        if velocity == 0: velocity = 1

        dodge_time = part_dist / (abs(velocity) + dodge_impulse(agent)) - 0.3

        vf = agent.me.velocity + agent.gravity * T
        local_vf = agent.me.local(vf.flatten())

        if 0.25 < T and T < 0.35 and (direction == -1 or agent.me.local_velocity().x < 900):
            agent.pop()
            local_flip_target = agent.me.local_location(agent.ball.location - (self.shot_vector * agent.ball_radius))
            agent.push(flip(local_flip_target, cancel=abs(Vector(x=1).angle2D(local_flip_target)) > 1.6))
        elif agent.me.airborne:
            agent.push(recovery(final_target if T > 0.5 else None))
        elif T <= -agent.delta_time * 6 or (T > 0.75 and distance_remaining > agent.me.hitbox.length / 2 + agent.ball_radius and not virxrlcu.ground_shot_is_viable(T, agent.boost_accel, agent.me.get_raw(agent), self.offset_target.z, tuple((final_target - agent.me.location).normalize()), distance_remaining)):
            # If we're out of time or not fast enough, we pop
            agent.pop()
            agent.shooting = False
            if agent.me.airborne:
                agent.push(recovery())
        elif dodge_time >= 1.2 and agent.time - agent.me.land_time > 0.5:
            if agent.me.boost < 48 and angle_to_target < 0.03 and velocity < speed_required - 50 and velocity - speed_required < dodge_impulse(agent) * 3:
                agent.push(flip(agent.me.local_location(self.offset_target)))
            elif direction == -1 and velocity < 200:
                agent.push(flip(agent.me.local_location(self.offset_target), True))


class generic_kickoff:
    def __init__(self):
        self.start_time = -1
        self.flip = False

    def run(self, agent: VirxERLU):
        if agent.cheating:
            ball_state = BallState(Physics(location=Vector3(0, 0, 92.75), velocity=Vector3(random.randint(0, 3000), random.randint(0, 3000), random.randint(0, 3000)), angular_velocity=Vector3(random.randint(0, 2), random.randint(0, 2), random.randint(0, 2))))
            agent.set_game_state(GameState(ball=ball_state))

            agent.kickoff_done = True
            agent.pop()
            return

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
        self.flip2 = None
        self.flip_done2 = None
        self.wait = -1

    def run(self, agent: VirxERLU):
        if self.start_time is None:
            self.start_time = agent.time

        agent.controller.throttle = 1

        if agent.me.boost > 22:
            agent.controller.boost = True

        if agent.me.boost > self.last_boost:
            if self.drive_right is None:
                self.drive_right = agent.time

        if self.drive_right is not None:
            if agent.time - self.drive_right < 0.125:
                agent.controller.steer = self.direction
            elif self.flip_done is None:
                if self.flip is None:
                    self.flip = flip(Vector(64, -36 * sign(agent.me.location.x * side(agent.team))))

                self.flip_done = self.flip.run(agent, manual=True, recovery_target=agent.ball.location.flatten())
                if abs(Vector(x=1).angle2D(agent.me.local_location(agent.ball.location))) < 0.05:
                    agent.controller.boost = agent.me.local_velocity().x < 2250
            elif self.wait == -1 or agent.time - self.wait < 0.05:
                if self.wait == -1:
                    self.wait = agent.time
                agent.controller.boost = agent.me.local_velocity().x < 2250
                return
            elif self.flip_done2 is None:
                if self.flip2 is None:
                    self.flip2 = flip(agent.me.local_location(agent.foe_goal.location))

                self.flip_done2 = self.flip2.run(agent, manual=True)
            else:
                agent.kickoff_done = True
                agent.pop()
                return

        self.last_boost = agent.me.boost

        if agent.time - self.start_time > 5:
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
        self.flip_predrive = None

    def run(self, agent: VirxERLU):
        if self.start_time is None:
            self.start_time = agent.time
            self.boost_pad = min(tuple(boost for boost in agent.boosts if not boost.large and boost.active and abs(boost.location.x) < 5 and abs(boost.location.y) < 3000), key=lambda boost: boost.location.flat_dist(agent.me.location))

        agent.controller.throttle = 1

        if self.do_boost:
            self.do_boost = agent.me.boost != 0
            agent.controller.boost = True

        if self.flip is None and agent.me.location.flat_dist(self.boost_pad.location + Vector(y=320 * side(agent.team))) > 380:
            defaultPD(agent, agent.me.local_location(self.boost_pad.location + Vector(y=320 * side(agent.team))))
        elif self.flip_done is None:
            if self.flip is None:
                self.boost_pad = tuple(boost for boost in agent.boosts if not boost.large and boost.active and abs(boost.location.x) < 5 and abs(boost.location.y) < 2000)
                if len(self.boost_pad) == 0:
                    agent.pop()
                    if agent.boost_amount == "default":
                        agent.push(back_offset_kickoff_boost())
                    else:
                        agent.push(retreat())
                        agent.kickoff_done = True
                    return

                self.boost_pad = min(self.boost_pad, key=lambda boost: boost.location.flat_dist(agent.me.location))
                self.flip = flip(Vector(27, 73 * sign(agent.me.location.x * side(agent.team))))

            self.flip_done = self.flip.run(agent, manual=True, recovery_target=agent.ball.location)
        elif self.flip_predrive is None or agent.time - self.flip_predrive <= 0.19:
            if self.flip_predrive is None:
                self.flip_predrive = agent.time

            defaultPD(agent, agent.me.local_location(agent.ball.location))
            agent.controller.throttle = 1
            agent.controller.boost = agent.me.local_velocity().x < 2250
        elif self.flip_done2 is None:
            if self.flip2 is None:
                self.flip2 = flip(agent.me.local_location(agent.ball.location))

            self.flip_done2 = self.flip2.run(agent, manual=True)
        else:
            agent.kickoff_done = True
            agent.pop()
            return

        if agent.time - self.start_time > 4:
            agent.kickoff_done = True
            agent.pop()


class back_kickoff:
    def __init__(self):
        self.t_start_time = None
        self.start_time = None
        self.start_boost_pad = None

    def run(self, agent):
        if self.t_start_time is None:
            self.t_start_time = agent.time
            self.start_boost_pad = min(tuple(boost for boost in agent.boosts if not boost.large and boost.active and abs(boost.location.x) < 5), key=lambda boost: boost.location.flat_dist(agent.me.location))

        agent.controller.throttle = 1

        if agent.time - self.t_start_time > 4:
            agent.kickoff_done = True
            agent.pop()

        if self.start_boost_pad.active:
            defaultPD(agent, agent.me.local_location(agent.ball.location))
            agent.controller.boost = True
            return

        if self.start_time is None:
            self.start_time = agent.time

        time_elapsed = round(agent.time - self.start_time, 5)

        if (0.0 <= time_elapsed <= 0.3) or (0.9417 <= time_elapsed <= 2.7333):
            agent.controller.throttle = 1
            agent.controller.pitch = -1

        if (0.0583 <= time_elapsed <= 0.1583) or (1 <= time_elapsed <= 1.1333) or (1.8917 <= time_elapsed <= 1.9833) or (2.0333 <= time_elapsed <= 2.1667):
            agent.controller.jump = True

        if (0.35 <= time_elapsed <= 0.75):
            agent.controller.throttle = -1
            agent.controller.pitch = 1

        if (0.0 <= time_elapsed <= 0.6417) or (1.05 <= time_elapsed <= 1.8917):
            agent.controller.boost = agent.me.local_velocity().x < 2290

        if time_elapsed > 2.7333:
            agent.pop()
            agent.push(recovery())
            agent.kickoff_done = True


class recovery:
    # Point towards our velocity vector and land upright, unless we aren't moving very fast
    # A vector can be provided to control where the car points when it lands
    def __init__(self, target=None):
        self.target = target

    def run(self, agent: VirxERLU):
        target = agent.me.velocity.normalize() if self.target is None else (self.target - agent.me.location).normalize()

        landing_plane = find_landing_plane(agent.me.location, agent.me.velocity, agent.gravity.z)

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
    # This routine drives towards the ball and attempts to hit it towards a given target, and can do it on walls!
    def __init__(self, target):
        self.target = target
        self.weight = -1
        self.eta = -1

        self.ball_location = None
        self.offset_target = None
        self.shot_vector = None
        self.last_touch = None

    def preprocess(self, agent: VirxERLU):
        ball_location = None
        dist_from_side = agent.ball_radius + agent.me.hitbox.height
        dist_from_ceiling = 2044 - dist_from_side
        dist_from_side_wall = 4096 - dist_from_side
        dist_from_back_wall = 5120 - dist_from_side

        for ball_slice in agent.ball_prediction_struct.slices[max(agent.min_intercept_slice-30, 0)::6]:
            time_remaining = ball_slice.game_seconds - agent.time
            location = Vector.from_vector(ball_slice.physics.location)

            # locate when the ball will be near some plane
            if time_remaining > 0 and agent.me.location.dist(location) / time_remaining < 2300:
                if abs(location.x) < dist_from_side_wall or abs(location.y) < dist_from_back_wall or location.z < dist_from_side or location.z > dist_from_ceiling:
                    self.eta = ball_slice.game_seconds
                    ball_location = location
                    break

        if ball_location is None:
            ball_location = Vector.from_vector(agent.ball_prediction_struct.slices[agent.ball_prediction_struct.num_slices - 1].physics.location)

        shot_vector = (self.target - agent.ball.location).normalize()

        if self.shot_vector is None or self.ball_location.dist(ball_location) > 5:
            self.ball_location = ball_location
            self.shot_vector = shot_vector
            self.offset_target = self.ball_location - (self.shot_vector * agent.ball_radius)

    def run(self, agent: VirxERLU):
        if not agent.shooting:
            agent.shooting = True

        if agent.odd_tick == 0 or self.ball_location is None:
            self.preprocess(agent)

        T = max(self.eta - agent.time, agent.delta_time)
        car_to_ball = (agent.ball.location - agent.me.location).normalize()
        final_target = agent.me.local_flatten(self.offset_target)

        angle_to_shot_vector = abs(car_to_ball.angle2D(self.shot_vector))
        if angle_to_shot_vector > no_adjust_radians:
            # whether we are to the left or right of the shot vector
            side_of_shot = sign(self.shot_vector.cross(Vector(z=1)).dot(car_to_ball))
            car_to_offset_target = final_target - agent.me.location
            car_to_offset_perp = car_to_offset_target.cross(Vector(z=side_of_shot)).normalize()  # perpendicular ray
            final_target += (car_to_offset_perp * (agent.me.local(car_to_ball).angle2D(agent.me.local(self.shot_vector)) * 1920)) if angle_to_shot_vector > min_adjust_radians else (car_to_offset_perp * agent.me.hitbox.width * 0.5)

        # Some adjustment to the final target to ensure we don't try to drive through any goalposts to reach it
        final_target = cap_in_field(agent, final_target)
        local_final_target_z = agent.me.local_location(self.offset_target).z
        local_final_target = agent.me.local_location(final_target)
        local_final_target.z = local_final_target_z
        distance_remaining = local_final_target.flatten().magnitude() + local_final_target_z
        distance_remaining -= agent.me.hitbox.length * 0.45
        if angle_to_shot_vector > no_adjust_radians:
            distance_remaining += agent.me.local(car_to_ball).angle2D(agent.me.local(self.shot_vector)) * 1920
        distance_remaining = max(distance_remaining, 0)

        agent.sphere(self.ball_location, agent.ball_radius, agent.renderer.black())
        agent.line(self.offset_target-Vector(z=agent.ball_radius), self.offset_target + Vector(z=agent.ball_radius), agent.renderer.green())
        agent.line(final_target-Vector(z=agent.ball_radius), final_target + Vector(z=agent.ball_radius), agent.renderer.purple())

        angles = defaultPD(agent, local_final_target)
        if Vector(x=1).angle2D(agent.me.local(self.offset_target)) < 0.1 and T < 1:
            speed_required = 2300
        else:
            speed_required = get_max_speed_from_local_point(local_final_target) if abs(agent.controller.steer) == 1 else min(distance_remaining / T, 2300)
        agent.dbg_2d(speed_required)
        # the angle to the final target, in radians
        angle_to_target = abs(Vector(x=1).angle2D(local_final_target))
        # whether we should go forwards or backwards
        direction = 1 if angle_to_target < 1.6 or speed_required > 1410 or (abs(speed_required) < 100 and angle_to_target < 1.7) else -1
        velocity = defaultThrottle(agent, speed_required * direction, angles, local_final_target)

        if velocity == 0: velocity = 1

        if agent.ball.last_touch.location.dist(agent.me.location) < agent.ball_radius + agent.me.hitbox.length * 0.6:
            agent.pop()
            agent.shooting = False
            agent.push(flip(agent.me.local(car_to_ball)))
        elif agent.me.airborne:
            agent.push(recovery())
        elif direction == 1 and distance_remaining < 320 and T < 1 and agent.time - agent.me.land_time > 0.5 and Vector(x=1).angle(agent.me.local_location(self.offset_target).flatten()) < 0.1:
            agent.push(flip(agent.me.local_location(self.offset_target)))
        elif (distance_remaining / (abs(velocity) + dodge_impulse(agent)) - 0.5 >= 1.2) and agent.time - agent.me.land_time > 0.5 and direction == -1 and velocity < 200:
            agent.push(flip(agent.me.local_location(self.offset_target), True))


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

        target = (agent.ball.location - agent.me.location).flatten().normalize() * 100
        target.z = -100
        target = agent.me.local(target)
        defaultPD(agent, target)
        if not agent.me.airborne:
            agent.pop()
        elif abs(Vector(x=1).angle(target)) < 0.5:
            agent.controller.boost = True
