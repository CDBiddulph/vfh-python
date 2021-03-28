from numpy.lib.arraypad import pad
from bike import Bike
from histogram_grid import HistogramGrid
from polar_histogram import PolarHistogram
from pure_pursuit import PPPathPlanner
from vfh_path_planner import VFHPathPlanner
from combined_path_planner import CombinedPathPlanner

import numpy as np


def test_vfh(vfh, robot_loc, target_loc):
    angle = vfh.get_best_angle(robot_loc, target_loc)
    return np.degrees(angle)


ACTIVE_REGION_DIM = 16
RESOLUTION = 1

histogram_grid = HistogramGrid.from_png_map("maps/map2_s.png", ACTIVE_REGION_DIM, RESOLUTION)

NUM_POLAR_BINS = 36
LOW_VALLEY_THRESHOLD = 1000
HIGH_VALLEY_THRESHOLD = LOW_VALLEY_THRESHOLD*1.2

polar_histogram = PolarHistogram(NUM_POLAR_BINS, LOW_VALLEY_THRESHOLD, HIGH_VALLEY_THRESHOLD)

STARTING_LOC = (24, 42)
STARTING_HEADING = 0
TARGET_LOC = (50, 50)

vfh = vfh_path_planner = VFHPathPlanner(histogram_grid, polar_histogram)
print(test_vfh(vfh, (24, 42), (50, 50)))
