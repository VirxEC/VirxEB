from typing import Optional

import numpy as np
import virx_erlu_rlib as rlru
from numba import njit
from util import routines, utils
from util.agent import Vector, VirxERLU
from util.routines import BaseRoutine


class GroundShot(routines.GroundShot):
    def __init__(self, shot_info: rlru.BasicShotInfo, target_id: int, weight: int):
        super().__init__(shot_info, target_id)
        self.weight = weight


class JumpShot(routines.JumpShot):
    def __init__(self, shot_info: rlru.BasicShotInfo, target_id: int, weight: int):
        super().__init__(shot_info, target_id)
        self.weight = weight


class DoubleJumpShot(routines.DoubleJumpShot):
    def __init__(self, shot_info: rlru.BasicShotInfo, target_id: int, weight: int):
        super().__init__(shot_info, target_id)
        self.weight = weight


class AerialShot(routines.AerialShot):
    def __init__(self, shot_info: rlru.BasicShotInfo, target_id: int, weight: int):
        super().__init__(shot_info, target_id)
        self.weight = weight


class ShortShot(routines.ShortShot):
    def __init__(self, target: Vector):
        super().__init__(target)
        self.weight = -1


class WaveDash(BaseRoutine):
    def __init__(self, target=None):
        self.step = -1
        # 0 = forward, 1 = left, 2 = backwards, 3 = right
        self.direction = 0
        self.start_time = -1

        if target is not None:
            self.direction = 0 if abs(target.x) > abs(target.y) else 1

            if (self.direction == 0 and target.x < 0) or (self.direction == 1 and target.y < 0):
                self.direction += 2

    def run(self, agent: VirxERLU):
        if self.start_time == -1:
            self.start_time = agent.time

        T = agent.time - self.start_time

        self.step += 1

        forward_target = agent.me.forward.flatten() * (agent.me.hitbox.length / 2)

        target_switch = {
            0: forward_target + Vector(z=25),
            1: forward_target,
            2: forward_target - Vector(z=25),
            3: forward_target
        }

        target_up = {
            0: Vector(z=1),
            1: (-agent.me.left.flatten() + Vector(z=1)).normalize(),
            2: Vector(z=1),
            3: (agent.me.left.flatten() + Vector(z=1)).normalize()
        }

        utils.defaultPD(agent, agent.me.local(target_switch[self.direction]), up=agent.me.local(target_up[self.direction]))
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
            agent.push(routines.Recovery())
        elif agent.me.location.z + (agent.me.velocity.z * 0.15) < 5:
            agent.controller.jump = True
            agent.controller.yaw = 0
            if self.direction in {0, 2}:
                agent.controller.roll = 0
                agent.controller.pitch = -1 if self.direction is 0 else 1
            else:
                agent.controller.roll = 1 if self.direction is 1 else -1
                agent.controller.pitch = 0


class FaceTarget(BaseRoutine):
    def __init__(self, target: Optional[Vector]=None, ball: bool=False):
        self.target = target
        self.ball = ball
        self.start_loc = None
        self.counter = 0

    @staticmethod
    def get_ball_target(agent: VirxERLU) -> Vector:
        ball = agent.ball.location if agent.me.minimum_time_to_ball == 7 else Vector(*rlru.get_slice_index(agent.min_intercept_slice).location)
        return ball.flatten()

    def run(self, agent: VirxERLU):
        if not agent.me.airborne and agent.time - agent.me.land_time < 0.5:
            return

        if self.ball:
            target = self.get_ball_target(agent) - agent.me.location
        else:
            target = agent.me.velocity if self.target is None else self.target - agent.me.location

        if agent.gravity.z < -550 and agent.gravity.z > -750:
            if self.counter == 0 and agent.me.airborne:
                self.counter = 3

            if self.counter < 3:
                self.counter += 1

            target = agent.me.local(target.flatten())
            if self.counter < 3:
                agent.controller.jump = True

            if agent.me.airborne:
                utils.defaultPD(agent, target)
            elif self.counter == 3:
                agent.pop()
        else:
            target = agent.me.local(target.flatten())
            angle_to_target = abs(Vector(x=1).angle(target))
            if angle_to_target > 0.1:
                if self.start_loc is None:
                    self.start_loc = agent.me.location

                direction = -1 if angle_to_target < 1.57 else 1

                agent.controller.steer = utils._fcap(target.y / 100, -1., 1.) * direction
                agent.controller.throttle = direction
                agent.controller.handbrake = True
            else:
                agent.pop()
                if self.start_loc is not None:
                    agent.push(routines.GoTo(self.start_loc, target, True))


class Shadow(BaseRoutine):
    def __init__(self):
        self.goto = routines.GoTo(Vector(), brake=True)

    def run(self, agent: VirxERLU):
        ball_loc = self.get_ball_loc(agent, True)
        target = self.get_target(agent, ball_loc)

        if self.switch_to_retreat(agent, ball_loc, target):
            agent.pop()
            agent.push(Retreat())
            return

        self_to_target = agent.me.location.flat_dist(target)

        if self_to_target < 100 * (agent.me.velocity.magnitude() / 500) and ball_loc.y < -640 and agent.me.velocity.magnitude() < 50 and abs(Vector(x=1).angle2D(agent.me.local_location(agent.ball.location))) > 1:
            agent.pop()
            if agent.num_friends > 1:
                agent.push(routines.FaceTarget(ball=True))
        else:
            self.goto.target = target
            self.goto.vector = ball_loc * Vector(y=utils.side(agent.team)) if target.y * utils.side(agent.team) < 1280 else None
            self.goto.run(agent)

    @staticmethod
    def switch_to_retreat(agent: VirxERLU, ball: Vector, target: Vector) -> bool:
        return agent.me.location.y * utils.side(agent.team) < ball.y or ball.y > 2560 or target.y * utils.side(agent.team) > 4480 or agent.is_own_goal

    @staticmethod
    def is_viable(agent: VirxERLU, ignore_distance: bool=False, ignore_retreat=False) -> bool:
        ball_loc = Shadow.get_ball_loc(agent)
        target = Shadow.get_target(agent, ball_loc)

        return (ignore_distance or agent.me.location.flat_dist(target) > 320) and (ignore_retreat or not Shadow.switch_to_retreat(agent, ball_loc, target))

    @staticmethod
    def get_ball_loc(agent: VirxERLU, render: bool=False) -> Vector:
        ball_slice = agent.ball.location
        ball_loc = Vector(ball_slice.x, ball_slice.y)
        if render: agent.sphere(ball_loc + Vector(z=agent.ball_radius), agent.ball_radius, color=agent.renderer.black())
        ball_loc.y *= utils.side(agent.team)

        if ball_loc.y < -2560 or (ball_loc.y < agent.ball.location.y * utils.side(agent.team)):
            ball_loc = Vector(agent.ball.location.x, agent.ball.location.y * utils.side(agent.team) - 640)

        return ball_loc

    @staticmethod
    def get_target(agent: VirxERLU, ball_loc: Optional[Vector]=None) -> Vector:
        if ball_loc is None:
            ball_loc = Shadow.get_ball_loc(agent)

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
            distance *= utils.cap(1 - (agent.enemy_time_to_ball / (8 - min(goal_diff, 6))), 0.5, 1) if goal_diff >= 0 else 1
            enemy_angle = abs(Vector(x=1).angle2D(agent.closest_foes[0].local_location(agent.ball.location)))
            if enemy_angle < 1:
                # factor in the enemy's speed - if they're travel at the ball at a high speed, then we need to back up because they're going to hit it hard
                distance *= min(agent.closest_foes[0].velocity.magnitude() - agent.me.velocity.magnitude() / 1000, 1 if agent.enemy_time_to_ball > 4 else 2)
            else:
                distance /= enemy_angle
            # make the minimum distance be 1920
            distance = max(distance, 1920)

        target = Vector(y=(ball_loc.y + distance) * utils.side(agent.team))
        if target.y * utils.side(agent.team) > -1280:
            # find the proper x coord for us to stop a shot going to the net
            # y = mx + b <- yes, finally! 7th grade math is paying off xD
            p1 = Retreat.get_target(agent)
            p2 = ball_loc * Vector(x=1, y=utils.side(agent.team))
            try:
                m = (p2.y - p1.y) / (p2.x - p1.x)
                b = p1.y - (m * p1.x)
                # x = (y - b) / m
                target.x = (target.y - b) / m
            except ZeroDivisionError:
                target.x = 0
        else:
            target.x = (abs(ball_loc.x) + 640) * utils._fsign(ball_loc.x)

        return Vector(target.x, target.y)


class Retreat(BaseRoutine):
    def __init__(self):
        self.goto = routines.GoTo(Vector(), brake=True)

    def run(self, agent: VirxERLU):
        ball = self.get_ball_loc(agent, render=True)
        target = self.get_target(agent, ball=ball)

        if Shadow.is_viable(agent, ignore_distance=True):
            agent.pop()
            agent.push(Shadow())
            return

        self.goto.target = target
        self.goto.run(agent)

    def is_viable(self, agent: VirxERLU) -> bool:
        return agent.me.location.flat_dist(self.get_target(agent)) > 320 and not Shadow.is_viable(agent, ignore_distance=True)

    @staticmethod
    def get_ball_loc(agent: VirxERLU, render: bool=False) -> Vector:
        ball_slice = agent.ball.location

        ball = Vector(ball_slice.x, ball_slice.y)
        if render: agent.sphere(ball + Vector(z=agent.ball_radius), agent.ball_radius, color=agent.renderer.black())
        ball.y *= utils.side(agent.team)

        if ball.y < agent.ball.location.y * utils.side(agent.team):
            ball = Vector(agent.ball.location.x, agent.ball.location.y * utils.side(agent.team) + 640)

        return ball

    @staticmethod
    def friend_near_target(agent: VirxERLU, target):
        for car in agent.friends:
            if car.location.dist(target) < 400:
                return True
        return False

    @staticmethod
    def get_target(agent: VirxERLU, ball: Optional[Vector]=None) -> Vector:
        if ball is None:
            ball = Retreat.get_ball_loc(agent)

        friends = np.array(tuple(friend.location._np for friend in agent.friends), dtype=np.float32, ndmin=2)

        friend_goal = np.array((
            agent.friend_goal.location._np,
            agent.friend_goal.left_post._np,
            agent.friend_goal.right_post._np
        ), dtype=np.float32)

        is_closest_ally = agent.me.minimum_time_to_ball < agent.friend_times[0].minimum_time_to_ball
        closest_foe_location = agent.closest_foes[0].location._np if agent.num_foes > 0 else np.array([0, 0, 0])

        return Vector(np_arr=Retreat._get_target(friends, friend_goal, ball._np, closest_foe_location, np.int32(agent.team), agent.is_own_goal, agent.num_foes, is_closest_ally))

    @staticmethod
    @njit('Array(float32, 1, "C")(Array(float32, 2, "C"), Array(float32, 2, "C"), Array(float32, 1, "C"), Array(float32, 1, "C"), int32, b1, int32, b1)', fastmath=True, cache=True)
    def _get_target(friends: np.ndarray, friend_goal: np.ndarray, ball: np.ndarray, closest_foe_location: np.ndarray, team: int, is_own_goal: bool, num_foes: int, is_closest_ally: bool) -> np.ndarray:
        target = None
        
        self_team = utils.side(team)
        num_friends = len(friends)

        horizontal_offset = 150
        outside_goal_offset = -125
        inside_goal_offset = 150

        if is_own_goal:
            target = np.array((ball[0], friend_goal[0][1], 0), dtype=np.float32)
        elif num_foes != 0 and (num_friends == 0 or is_closest_ally):
            real_ball = ball * np.array((0, self_team, 0), dtype=np.float32)
            s = real_ball - closest_foe_location
            s /= np.linalg.norm(s)
            start = friend_goal[1] - real_ball
            start /= np.linalg.norm(start)
            end = friend_goal[2] - real_ball
            end /= np.linalg.norm(end)

            neg_one = np.array((0, 0, -1), dtype=np.float32)
            right = s.dot(np.cross(end, neg_one)) < 0
            left = s.dot(np.cross(start, neg_one)) > 0

            if (right and left) if end.dot(np.cross(start, neg_one)) > 0 else (right or left):
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
            elif start.dot(s) < end.dot(s):
                target = friend_goal[2].copy()
            else:
                target = friend_goal[1].copy()
        else:
            if ball[1] < -640:
                target = friend_goal[0].copy()
            elif ball[0] * self_team < friend_goal[2][0] * self_team:
                target = friend_goal[2].copy()

                while utils.friend_near_target(friends, target):
                    target[0] = (target[0] * self_team + horizontal_offset * self_team) * self_team
            elif ball[0] * self_team > friend_goal[1][0] * self_team:
                target = friend_goal[1].copy()

                while utils.friend_near_target(friends, target):
                    target[0] = (target[0] * self_team - horizontal_offset * self_team) * self_team
            else:
                target = friend_goal[0].copy()
                target[0] = ball[0]

                while utils.friend_near_target(friends, target):
                    target[0] = (target[0] * self_team - horizontal_offset * utils._fsign(ball[0]) * self_team) * self_team

        target[1] += (inside_goal_offset if abs(target[0]) < 800 else outside_goal_offset) * self_team
        target[2] = 0

        return target


class CornerKickoff(BaseRoutine):
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
                    self.flip = routines.Flip(Vector(64, -36 * utils._fsign(agent.me.location.x * utils.side(agent.team))), agent.ball.location.flatten())

                self.flip_done = self.flip.run(agent, manual=True)
                if abs(Vector(x=1).angle2D(agent.me.local_location(agent.ball.location))) < 0.05:
                    agent.controller.boost = agent.me.local_velocity().x < 2250
            elif self.wait == -1 or agent.time - self.wait < 0.05:
                if self.wait == -1:
                    self.wait = agent.time
                agent.controller.boost = agent.me.local_velocity().x < 2250
                return
            elif self.flip_done2 is None:
                if self.flip2 is None:
                    self.flip2 = routines.Flip(agent.me.local_location(agent.foe_goal.location))

                self.flip_done2 = self.flip2.run(agent, manual=True)
            else:
                agent.kickoff_done = True
                agent.pop()
                return

        self.last_boost = agent.me.boost

        if agent.time - self.start_time > 5:
            agent.kickoff_done = True
            agent.pop()


class CornerKickoffBoost(BaseRoutine):
    def __init__(self):
        self.goto = False

    def run(self, agent: VirxERLU):
        if not self.goto:
            self.goto = True
            agent.push(routines.GoToBoost(min(tuple(boost for boost in agent.boosts if boost.large and boost.active), key=lambda boost: boost.location.flat_dist(agent.me.location))))
        else:
            agent.pop()
            agent.kickoff_done = True


class BackOffsetKickoffBoost:
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
                self.goto = routines.GoTo(self.small_boost.location, self.large_boost.location)

            self.small = not self.small_boost.active
            self.goto.run(agent, manual=True)
            agent.controller.boost = True
        elif not self.large:
            agent.push(routines.GoToBoost(self.large_boost))
            self.large = True
        else:
            agent.pop()
            agent.kickoff_done = True


class BackOffsetKickoff:
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

        if self.flip is None and agent.me.location.flat_dist(self.boost_pad.location + Vector(y=320 * utils.side(agent.team))) > 380:
            utils.defaultPD(agent, agent.me.local_location(self.boost_pad.location + Vector(y=320 * utils.side(agent.team))))
        elif self.flip_done is None:
            if self.flip is None:
                self.boost_pad = tuple(boost for boost in agent.boosts if not boost.large and boost.active and abs(boost.location.x) < 5 and abs(boost.location.y) < 2000)
                if len(self.boost_pad) == 0:
                    agent.pop()
                    if agent.boost_amount == "default":
                        agent.push(BackOffsetKickoffBoost())
                    else:
                        agent.push(routines.Retreat())
                        agent.kickoff_done = True
                    return

                self.boost_pad = min(self.boost_pad, key=lambda boost: boost.location.flat_dist(agent.me.location))
                self.flip = routines.Flip(Vector(27, 73 * utils._fsign(agent.me.location.x * utils.side(agent.team))), agent.ball.location.flatten())

            self.flip_done = self.flip.run(agent, manual=True)
        elif self.flip_predrive is None or agent.time - self.flip_predrive <= 0.19:
            if self.flip_predrive is None:
                self.flip_predrive = agent.time

            utils.defaultPD(agent, agent.me.local_location(agent.ball.location))
            agent.controller.throttle = 1
            agent.controller.boost = agent.me.local_velocity().x < 2250
        elif self.flip_done2 is None:
            if self.flip2 is None:
                self.flip2 = routines.Flip(agent.me.local_location(agent.ball.location))

            self.flip_done2 = self.flip2.run(agent, manual=True)
        else:
            agent.kickoff_done = True
            agent.pop()
            return

        if agent.time - self.start_time > 4:
            agent.kickoff_done = True
            agent.pop()


class BackKickoff:
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
            utils.defaultPD(agent, agent.me.local_location(agent.ball.location))
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
            agent.push(routines.Recovery())
            agent.kickoff_done = True

class BoostDown(BaseRoutine):
    def __init__(self):
        self.face = routines.BallRecovery()

    def run(self, agent: VirxERLU):
        if agent.me.boost == 0:
            agent.pop()
            agent.push(self.face)

        target = (agent.ball.location - agent.me.location).flatten().normalize() * 100
        target.z = -100
        target = agent.me.local(target)
        utils.defaultPD(agent, target)
        if not agent.me.airborne:
            agent.pop()
        elif abs(Vector(x=1).angle(target)) < 0.5:
            agent.controller.boost = True