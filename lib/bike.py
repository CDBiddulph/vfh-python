import math
import numpy as np
import queue
from vfh_path_planner import VFHPathPlanner

# Used for the simulation
SMALL_DISTANCE = 0.01
SMALL_ANGLE = 0.01


class Bike:
    def __init__(self, vfh_path_planner, start_pos, target_pos, heading, max_speed=5, max_accel=0.3, curve_const=3, yaw_adjust_speed=50, vfh_lookahead_dist=25):
        """
        x = starting x-coordinate, in m (+x points to the right)
        y = starting y-coordinate, in m (+y points upwards)
        heading = starting header, in radians (0 is +x axis, increases counterclockwise)
        """
        self.pos = start_pos
        self.target_pos = target_pos
        self.heading = heading
        self.speed = 0
        self.yaw = 0
        self.step_count = 0

        self.MAX_SPEED = max_speed
        self.MAX_ACCEL = max_accel
        self.CURVE_CONST = curve_const
        self.YAW_ADJUST_SPEED = yaw_adjust_speed

        self.vfh_path_planner = vfh_path_planner

        self.VFH_LOOKAHEAD_DIST = vfh_lookahead_dist

    def step(self, speed, yaw_dot):
        """Perform one simulation step of the bike"""
        old_x, old_y = self.pos
        self.pos = (old_x + SMALL_DISTANCE * speed * math.cos(self.heading),
                    old_y + SMALL_DISTANCE * speed * math.sin(self.heading))

        self.heading += yaw_dot * SMALL_ANGLE

    def get_nav_command(self):
        """Returns the pair (speed, yaw_dot)"""
        self.speed = self.MAX_SPEED
        best_angle = self.vfh_path_planner.get_best_angle(self.pos, self.target_pos)

        vfh_lookahead_x = self.pos[0] + self.VFH_LOOKAHEAD_DIST * math.cos(best_angle)
        vfh_lookahead_y = self.pos[1] + self.VFH_LOOKAHEAD_DIST * math.sin(best_angle)
        self.update_yaw(vfh_lookahead_x, vfh_lookahead_y)

        self.step_count += 1
        return self.speed, self.yaw

    def update_yaw(self, follow_x, follow_y):
        """Returns the pair (speed, yaw_dot)"""
        reverse_x, reverse_y = self.get_reverse_point(follow_x, follow_y)
        curvature = get_curvature(follow_x, follow_y, self.pos[0], self.pos[1], reverse_x, reverse_y)
        sign = -math.sin(self.heading - math.atan2(follow_y - self.pos[1], follow_x - self.pos[0]))
        # just shake up the yaw a bit if follow_point is right behind/ahead of self, to check
        self.yaw = math.copysign(self.YAW_ADJUST_SPEED * curvature, sign) if sign ** 2 > 0.0000000001 else 0.01

    def get_reverse_point(self, follow_x, follow_y):
        dx = follow_x - self.pos[0]
        dy = follow_y - self.pos[1]
        magnitude = math.sqrt(dx ** 2 + dy ** 2)
        follow_heading = math.atan2(dy, dx)
        # the reverse heading formula is simplified from the more-intuitive formula:
        # follow_heading + 2(pi/2 - (follow_heading-self.heading))
        reverse_heading = math.pi + 2 * self.heading - follow_heading
        reverse_x = self.pos[0] + magnitude * math.cos(reverse_heading)
        reverse_y = self.pos[1] + magnitude * math.sin(reverse_heading)
        return reverse_x, reverse_y

    def get_obstacles(self):
        return self.vfh_path_planner.histogram_grid.get_obstacles()


def get_curvature(x1, y1, x2, y2, x3, y3):
    very_small = 0.00000000000000000001
    x1_minus_x2 = x1 - x2
    if x1_minus_x2 == 0:
        x1_minus_x2 = very_small  # avoids division by zero
    k1 = 0.5 * (x1 ** 2 + y1 ** 2 - x2 ** 2 - y2 ** 2) / x1_minus_x2
    k2 = (y1 - y2) / x1_minus_x2
    b_denom = (x3 * k2 - y3 + y2 - x2 * k2)
    if b_denom == 0:
        b_denom = very_small  # avoids division by zero
    b = 0.5 * (x2 ** 2 - 2 * x2 * k1 + y2 ** 2 - x3 ** 2 + 2 * x3 * k1 - y3 ** 2) / b_denom
    a = k1 - k2 * b
    raised_to_negative_half = ((x1 - a) ** 2 + (y1 - b) ** 2)
    if raised_to_negative_half == 0:
        raised_to_negative_half = very_small  # avoids division by zero
    return raised_to_negative_half ** (-0.5)
