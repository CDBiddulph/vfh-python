"""
path_planner.py

PathPlanner should cannibalize both histogram_grid and polar_histogram. There
is no reason that
"""
import warnings
import math
from itertools import groupby
from operator import itemgetter

# PolarHistogram class creates an object to represent the Polar Histogram


class PathPlanner:
    def __init__(self, histogram_grid, polar_histogram, robot_location, target_location, a=200, b=1, l=5,
                 s_max=15, valley_threshold=200):
        """
        Creates a Polar Histogram object with the number of bins passed.

        Args:
            polar_histogram: Object used to store the polar histogram.
            histogram_grid: Object used to store the grid/map of obstacles.
            a, b, l: Hyperparameters for the smoothing the polar histogram.
            s_max: Hyperparameter: the maximum number of nodes that define a wide valley
        """
        self.polar_histogram = polar_histogram
        self.histogram_grid = histogram_grid
        self.a = a
        self.b = b
        self.l = l
        self.s_max = s_max
        self.valley_threshold = valley_threshold
        self.target_location = target_location
        self.robot_location = robot_location

#   TODO: Add ability to dynamically set certainty value

    def set_robot_location(self, loc):
        self.robot_location = loc
        self.generate_histogram(loc)

    def set_target_discrete_location(self, target_discrete_location):
        self.histogram_grid.set_target_discrete_location(
            target_discrete_location)

    def generate_histogram(self, robot_location):
        """Builds the vector field histogram based on current position of robot and surrounding obstacles"""
        self.polar_histogram.reset()

        # print("path_planner: generate_histogram: robot_location =", robot_location)
        active_region_min_x, active_region_min_y, active_region_max_x, active_region_max_y = self.histogram_grid.get_active_region(
            robot_location)
        # print("path_planner: generate_histogram: active_region =", (active_region_min_x, active_region_min_y, active_region_max_x, active_region_max_y))
        histogram_grid = self.histogram_grid
        polar_histogram = self.polar_histogram

        for x in range(active_region_min_x, active_region_max_x):
            for y in range(active_region_min_y, active_region_max_y):
                node_considered = (x, y)
                certainty = histogram_grid.get_certainty_at_discrete_point(
                    node_considered)
                distance = histogram_grid.get_continuous_distance_between_discrete_points(
                    node_considered, robot_location)
                delta_certainty = (certainty ** 2) * \
                    (self.a - self.b * distance)
                robot_to_node_angle = histogram_grid.get_angle_between_discrete_points(
                    robot_location, node_considered)
                # print("path_planner: robot_to_node_angle between robot_location", robot_location,
                #       "and node_considered", node_considered, "is", robot_to_node_angle)
                if delta_certainty != 0:
                    # print("path_planner: adding certainty %.1f to angle %s or node" % (delta_certainty, robot_to_node_angle), node_considered)
                    polar_histogram.add_certainty_to_bin_at_angle(
                        robot_to_node_angle, delta_certainty)

        polar_histogram.smooth_histogram(self.l)

    # TODO: We need to reorganize the polar histogram, starting NOT with the
    # target angle but with first bin closest to the target angle which does
    # not have a certainty (due to distance) and ending at max length.

    def get_filtered_polar_histogram(self):
        filtered = [bin_index for bin_index, certainty
                    in enumerate(self.polar_histogram._polar_histogram)
                    if certainty < self.valley_threshold]
        return filtered

    def get_sectors(self):
        filtered_polar_histogram = self.get_filtered_polar_histogram()
        num_bins = self.polar_histogram.num_bins
        # return early if every sector is under or over the threshold
        if num_bins == len(filtered_polar_histogram):
            return [(0, num_bins - 1)]
        elif num_bins == 0:
            return []
        sectors = []
        last_bin = start_bin = filtered_polar_histogram[0]

        for bin in filtered_polar_histogram[1:]:
            # if a new bin is starting
            if last_bin + 1 != bin:
                sectors.append((start_bin, last_bin))
                start_bin = bin
            last_bin = bin

        if last_bin == num_bins - 1 and sectors and sectors[0][0] == 0:
            sectors[0] = (start_bin, sectors[0][1])
        else:
            sectors.append((start_bin, last_bin))

        return sectors

    def get_obstacles(self):
        return self.histogram_grid.get_obstacles()

    def get_best_angle(self, robot_to_target_angle):
        sectors = self.get_sectors()
        num_bins = self.polar_histogram.num_bins
        if len(sectors) == 0:
            least_likely_bin = sorted(range(len(self.polar_histogram._polar_histogram)),
                                      key=lambda k: self.polar_histogram._polar_histogram[k])[0]
            middle_angle = self.polar_histogram.get_middle_angle_of_bin(
                least_likely_bin)
            warnings.warn("path_planner: the entire polar histogram is above valley threshold = %s, setting best angle to least likely bin middle angle = %s" % (
                self.valley_threshold, middle_angle))
            return middle_angle

        angles = []
        for sector in sectors:
            # counterclockwise from ang1, you'll hit middle_angle, then ang2
            print(sector)
            if sector[0] > sector[1]:
                sector = (sector[0], sector[1] + num_bins)
            sector_width = (sector[1] - sector[0]) + 1
            print(sector_width, self.s_max)

            if sector_width > self.s_max:
                # Case 1: Wide valley. Include only s_max bins.
                k1, k2 = sector[0] + sector_width/

            else:
                # Case 2: Narrow valley. Include all bins.
                ang1 = self.polar_histogram.get_middle_angle_of_bin(sector[0])
                ang2 = self.polar_histogram.get_middle_angle_of_bin(sector[1])
                print(ang1, ang2)
                middle_angle = ang1 + (ang2-ang1) / 2
                angles.append(middle_angle)

        print([(a, abs(robot_to_target_angle - a)) for a in angles])

        result = min(angles, key=lambda ang: abs(robot_to_target_angle - ang))
        print(result)
        return result

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
