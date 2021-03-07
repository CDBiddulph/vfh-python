import csv
import math
from geom_util import get_discrete_displacement
import numpy as np
import imageio


class HistogramGrid:
    def __init__(self, dimension, resolution, active_region_dimension):
        """
        dimension: Number of cells on one side.
        resolution: Size of the cell in centimeters.
        """
        # not sure if I should do this
        # assert(active_region_dimension % 2 == 1, "active_region_dimension should be odd")

        self.dimension = dimension
        self.resolution = resolution
        ncols, nrows = dimension
        self.histogram_grid = [[0] * ncols for r in range(nrows)]
        self.active_region_dimension = active_region_dimension

    @classmethod
    def from_txt_map(cls, map_fname, active_region_dimension, resolution):
        """
        Args:
            active_region_dimension: the number of cells on either side of the active region
        """
        with open(map_fname, 'r') as f:
            reader = csv.reader(f, delimiter=" ")
            lines = list(reader)

        # Convert string "1"s and "0"s to integer 1s and 0s
        lines = list(map(lambda l: list(map(int, l)), lines))
        lines = np.asarray(lines)
        dimension = (len(lines[0]), len(lines))
        hg = cls(dimension, resolution, active_region_dimension)
        hg.histogram_grid = lines
        return hg

    @classmethod
    def from_png_map(cls, map_fname, active_region_dimension, resolution):
        lines = np.asarray(imageio.imread(map_fname))
        lines = np.average(-lines + 255, axis=2)
        lines = lines*100//255
        dimension = lines.shape
        hg = cls(dimension, resolution, active_region_dimension)
        hg.histogram_grid = lines
        return hg

    def continuous_point_to_discrete_point(self, continuous_point):
        """
        Calculates in which node an object exists based on the continuous (exact) coordinates
        Returns a discrete point
        Args:
            continuous_point: A tuple ()
        """
        continuous_x, continuous_y = continuous_point
        discrete_x = continuous_x//self.resolution
        discrete_y = continuous_y//self.resolution
        # discrete_x = continuous_x//self.dimension[0]
        # discrete_y = continuous_y//self.dimension[1]
        # if(out.x < iMax && out.y < jMax) return out;
        # TODO ERROR HANDLING
        # throw;
        return discrete_x, discrete_y

    def discrete_point_to_continous_point(self, discrete_point):
        discrete_x, discrete_y = discrete_point
        continuous_x = discrete_x*self.resolution
        continuous_y = discrete_y*self.resolution
        return continuous_x, continuous_y

    def update_certainty_at_continuous_point(self, continuous_point, certainty):
        """
        Updates the certainty value for the node at which the object is located.

        Args:
            continuous_point: the continuous point at which object is located.
            certainty: certainty value to set.
        """
        discrete_x, discrete_y = self.continuous_point_to_discrete_point(
            continuous_point)
        # self.histogram_grid[discrete_x][discrete_y] = certainty
        self.histogram_grid[discrete_y][discrete_x] = certainty

    def get_certainty_at_discrete_point(self, discrete_point):
        """
        Returns the certainty of an object being present at the given node
        """
        discrete_x, discrete_y = discrete_point
        try:
            return self.histogram_grid[discrete_y][discrete_x]
        except IndexError:
            return 0

    def get_continuous_distance_between_discrete_points(self, discrete_start, discrete_end):
        """
        Returns scalar distance between two discretePoints (pos1 & pos2) on the histogram grid
        """
        discrete_displacement = get_discrete_displacement(
            discrete_start, discrete_end)
        continuous_displacement = tuple(
            self.resolution * axis for axis in discrete_displacement)
        continuous_distance = math.sqrt(
            sum(axis**2 for axis in continuous_displacement))
        return continuous_distance

    def get_active_region(self, robot_location):
        robot_location_x, robot_location_y = robot_location

        active_region_min_x = robot_location_x - self.active_region_dimension / 2
        active_region_min_y = robot_location_y - self.active_region_dimension / 2
        active_region_max_x = robot_location_x + self.active_region_dimension / 2
        active_region_max_y = robot_location_y + self.active_region_dimension / 2

        # TODO: is rounding the best way to do this?
        return (int(round(active_region_min_x)), int(round(active_region_min_y)), int(
            round(active_region_max_x)), int(round(active_region_max_y)))

    # CHANGED: Make Robot class the sole source of truth for location
    # def get_robot_location(self):
    #     return self.robot_location

    def get_target_discrete_location(self):
        return self.target_discrete_location

    # CHANGED: Make Robot class the sole source of truth for location
    # def set_robot_location(self, robot_location):
    #     self.robot_location = robot_location

    def set_target_discrete_location(self, target_discrete_location):
        self.target_discrete_location = target_discrete_location

    # Get points(tuples)
    def get_obstacles(self):

        obstacles_points_x = []
        obstacles_points_y = []
        obstacles_points_prob = []
        for row_idx, row in enumerate(self.histogram_grid):
            for col_idx, cell in enumerate(row):
                if cell != 0:
                    # print("histogram_grid: obstacle =", (row_idx, col_idx))
                    obstacles_points_x.append(col_idx)
                    obstacles_points_y.append(row_idx)
                    obstacles_points_prob.append(cell)
        return [(*self.discrete_point_to_continous_point((x, y)), prob)
                for x, y, prob in zip(obstacles_points_x, obstacles_points_y, obstacles_points_prob)]

    def get_histogram_grid_active_region(self, active_region_min_x, active_region_min_y, active_region_max_x, active_region_max_y):
        # Return histogram_grid[active_region_min_x:active_region_max_y]
        return [self.histogram_grid[row][active_region_min_x:active_region_max_x + 1] for row in range(active_region_min_y, active_region_max_y + 1)]
        # return [self.histogram_grid[row][active_region_min_y:active_region_max_y + 1] for row in range(active_region_min_x, active_region_max_x + 1)]

    def get_shape(self):
        return self.histogram_grid.shape

    def get_real_size(self):
        return self.dimension[0] * self.resolution, self.dimension[1] * self.resolution
