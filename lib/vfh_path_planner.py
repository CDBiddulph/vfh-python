import warnings
import math
from itertools import groupby
from operator import itemgetter
from geom_util import get_angle_between_points

import numpy as np


class VFHPathPlanner:
    def __init__(self, histogram_grid, polar_histogram, l=5, s_max=15, min_cell_dist=5, angle_cost_weights=(5, 2, 2)):
        """
        Creates a Polar Histogram object with the number of bins passed.

        Args:
            polar_histogram: Object used to store the polar histogram.
            histogram_grid: Object used to store the grid/map of obstacles.
            l: Hyperparameter for smoothing the polar histogram.
            s_max: Hyperparameter: the maximum number of nodes that define a wide valley
        """
        self.polar_histogram = polar_histogram
        self.histogram_grid = histogram_grid
        self.l = l
        self.s_max = s_max
        # right now, just used in polar histogram mask
        self.min_cell_dist = min_cell_dist
        # represent weights for 1) angle from goal, 2) angle from heading,
        # and 3) angle from last chosen direction
        self.angle_cost_weights = angle_cost_weights

        self.calculate_a_and_b()

        self.last_direction = None

    def calculate_a_and_b(self):
        d_max = math.sqrt(2) * (self.histogram_grid.active_region_dimension - 1)/2
        self.a = 1
        self.b = self.a / d_max

    def set_robot_location(self, loc):
        self.robot_location = loc

    def set_target_discrete_location(self, target_discrete_location):
        self.histogram_grid.set_target_discrete_location(
            target_discrete_location)

    def generate_histogram(self, robot_location, robot_direction):
        """Builds the vector field histogram based on current position of robot and surrounding obstacles"""
        self.polar_histogram.reset()

        ar_min_x, ar_min_y, ar_max_x, ar_max_y = self.histogram_grid.get_active_region(
            robot_location)

        for x in range(ar_min_x, ar_max_x):
            for y in range(ar_min_y, ar_max_y):
                node_considered = (x, y)
                certainty = self.histogram_grid.get_certainty_at_discrete_point(node_considered)
                distance = self.histogram_grid.get_continuous_distance_between_discrete_points(
                    node_considered, robot_location)
                delta_certainty = (certainty ** 2) * (self.a - self.b * distance)
                robot_to_node_angle = get_angle_between_points(robot_location, node_considered)
                if delta_certainty != 0:
                    self.polar_histogram.add_certainty_to_bin_at_angle(
                        robot_to_node_angle, delta_certainty)

        self.polar_histogram.smooth_histogram(self.l)

        coords_over_tau = self.histogram_grid.get_coordinates_over_tau(
            ar_min_x, ar_max_x, ar_min_y, ar_max_y)
        self.polar_histogram.mask(coords_over_tau, robot_location, robot_direction, self.min_cell_dist)

    def get_best_angle(self, robot_loc, robot_dir, target_loc):
        self.generate_histogram(robot_loc, robot_dir)

        sectors = self.polar_histogram.get_sectors()
        num_bins = self.polar_histogram.get_num_bins()
        bin_width = self.polar_histogram.bin_width
        half_s_max_angle = self.s_max*bin_width/2
        if len(sectors) == 0:
            least_likely_bin = min(range(num_bins), key=lambda k: self.polar_histogram.get(k))
            middle_angle = self.polar_histogram.get_middle_angle_of_bin(least_likely_bin)
            warnings.warn("path_planner: the entire polar histogram is occupied, setting \
                best angle to least likely bin middle angle = %s" % (middle_angle))
            return middle_angle

        robot_to_target_angle = get_angle_between_points(robot_loc, target_loc)
        angles = []
        for bin_r, bin_l in sectors:
            # counterclockwise from bin_r, you'll hit middle_angle, then bin_l
            if bin_r > bin_l:
                bin_l += num_bins
            sector_diff = bin_l - bin_r
            # edge case: if all bins are available, move toward target
            if sector_diff >= num_bins - 1:
                return robot_to_target_angle

            s_l = (bin_l + 0.5) * bin_width
            s_r = (bin_r + 0.5) * bin_width

            if sector_diff > self.s_max:
                # Case 1: Wide valley. Include only s_max bins.
                padded_s_l = s_l - half_s_max_angle
                padded_s_r = s_r + half_s_max_angle
                angles.append(padded_s_l)
                angles.append(padded_s_r)
                # determine if robot_to_target_angle is between the two
                while padded_s_l > robot_to_target_angle and padded_s_r > robot_to_target_angle:
                    padded_s_l -= 2*math.pi
                    padded_s_r -= 2*math.pi
                while padded_s_l < robot_to_target_angle and padded_s_r < robot_to_target_angle:
                    padded_s_l += 2*math.pi
                    padded_s_r += 2*math.pi
                if padded_s_l > robot_to_target_angle > padded_s_r:
                    angles.append(robot_to_target_angle)
            else:
                # Case 2: Narrow valley. Include all bins.
                middle_angle = s_l + (s_r-s_l) / 2
                angles.append(middle_angle)

        # print([np.degrees(a) for a in angles])
        result = min(angles, key=lambda a: self.angle_cost(a, robot_to_target_angle, robot_dir))
        self.last_direction = result
        return result

    def angle_cost(self, ang, robot_to_target, heading):
        t1 = self.angle_cost_weights[0]*angle_distance(ang, robot_to_target)
        t2 = self.angle_cost_weights[1]*angle_distance(ang, heading)
        # account for the fact that the first timestep will have no last_direction
        t3 = self.angle_cost_weights[2]*angle_distance(ang, self.last_direction) \
            if self.last_direction is not None else 0
        return t1 + t2 + t3

    def print_histogram(self):
        print(self.polar_histogram)

    def get_object_grid(self):
        return self.histogram_grid.get_object_grid()

    def get_cell_value(self, i, j):
        return self.histogram_grid.get_cell_value(i, j)

    def get_i_max(self):
        return self.histogram_grid.get_i_max()

    def get_j_max(self):
        return self.histogram_grid.get_j_max()

    def get_histogram_grid(self):
        return self.histogram_grid

    def get_polar_histogram(self):
        return self.polar_histogram


# cosine is effective for ranking how different angles are
def angle_distance(a1, a2):
    return -math.cos(a1 - a2) + 1
