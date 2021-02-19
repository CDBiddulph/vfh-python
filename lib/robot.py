# /*
# * NOTE: it is important to distinguish between the same variables at
# * t versus t-1. Instance variables are shared across two timesteps.
# */

# NOTE: all speed and velocity units are continuous distance per timestep.tgm
import matplotlib.patches as patches
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import math
import numpy as np
from IPython import display
import itertools


from .path_planner import PathPlanner
from .histogram_grid import HistogramGrid
from .polar_histogram import PolarHistogram


class Robot:
    def __init__(self, histogram_grid, polar_histogram, init_location, target_location, init_speed):
        # CHANGED: we shouldn't need polar_histogram, only histogram_grid
        self.path_planner = PathPlanner(histogram_grid, polar_histogram)
        self.location = init_location
        self.target_location = target_location
        self.speed = init_speed
        self.update_angle()

    @classmethod
    def from_map(cls, map_fname, is_txt, init_location, target_location, init_speed, active_region_dimension, resolution, num_bins):
        histogram_grid = HistogramGrid.from_txt_map(map_fname, active_region_dimension, resolution) \
            if is_txt else HistogramGrid.from_png_map(map_fname, active_region_dimension, resolution)

        polar_histogram = PolarHistogram(num_bins)
        hist_shape = histogram_grid.get_shape()
        target_location = (target_location[0]*hist_shape[0], target_location[1]*hist_shape[1])
        return cls(histogram_grid, polar_histogram, init_location, target_location, init_speed)

    def update_angle(self):
        self.move_angle = self.path_planner.get_best_angle(self.location, self.target_location)

    def set_speed(self, speed):
        self.speed = speed

    def update_velocity(self):
        # old_v_x, old_v_y = self.velocity
        self.velocity = (self.speed * math.cos(self.move_angle),
                         self.speed * math.sin(self.move_angle))

    def update_location(self):
        velocity_x, velocity_y = self.velocity
        old_x, old_y = self.location
        self.location = (old_x + velocity_x, old_y + velocity_y)

    def get_location(self):
        return self.path_planner.robot_location

    # Main function per timestep
    # 1. Get angle from nothing at t=0, then
    # 2. get speed from nothing at t=0.
    # 3. Given position at 0, draw simulation at t=0,
    # 4. Now move from t=0 to t=1 by only updating the robot's position.

    def step(self, draw=True):
        # self.print_histogram()
        self.update_angle()  # angle: Null (or optionally, t-1) => t
        # self.set_speed() # speed: Null (or optionally, t-1) => t
        # print("robot: step: best angle =", self.move_angle )
        self.update_velocity()
        self.update_location()  # position: t => t+1

    def loop(self, draw=True):
        plt.ion()  # enable interactive plotting mode

        if draw == True:
            figure, (simulation_plot, polar_plot, histogram_grid_plot) = plt.subplots(
                1, 3, figsize=(18, 6))

            # 1. Plot the simulation
            # get a list of points [(x1, y1), (x2, y2), ...]
            obstacles_x, obstacles_y = self.path_planner.histogram_grid.get_obstacles()
            simulation_plot.scatter(*self.location, color='blue')
            simulation_plot.scatter(*self.target_location, color='green')
            simulation_plot.scatter(obstacles_x, obstacles_y, color='red')
            active_region_min_x, active_region_min_y, active_region_max_x, active_region_max_y = \
                self.path_planner.histogram_grid.get_active_region(self.location)
            rectangle = simulation_plot.add_patch(
                patches.Rectangle(
                    (active_region_min_x, active_region_min_y),
                    active_region_max_x - active_region_min_x,
                    active_region_max_y - active_region_min_y,
                    fill=False
                )
            )

            # 2. Plot the polar histogram
            num_bins = self.path_planner.polar_histogram.num_bins
            valley_threshold = self.path_planner.valley_threshold
            polar_histogram_by_angle = self.path_planner.polar_histogram.get_angle_certainty()
            # NOTE: instead of sectors, get polar histogram bins and filter them by valley threshold
            bin_percentages = [1.0/num_bins for angle,
                               certainty in polar_histogram_by_angle]
            bin_certainties = [certainty for angle,
                               certainty in polar_histogram_by_angle]
            colors = ['blue' if certainty < valley_threshold else 'red' for angle,
                      certainty in polar_histogram_by_angle]
            labels = [round(np.rad2deg(angle)) for angle, certainty in polar_histogram_by_angle]
            generator = enumerate(polar_histogram_by_angle)

            def make_autopct(bin_percentages):
                def my_autopct(pct):
                    index, (angle, certainty) = next(generator)
                    return '{angle:.0f}:  {certainty:.1f}'.format(angle=angle, certainty=certainty)
                return my_autopct

            pie_patches, pie_texts, pie_autotexts = polar_plot.pie(
                bin_percentages, colors=colors, labels=labels, startangle=0, counterclock=True, autopct=make_autopct(bin_percentages))

            # 3. Plot the valley
            histogram_grid_active_region = self.path_planner.histogram_grid.get_histogram_grid_active_region(
                active_region_min_x, active_region_min_y, active_region_max_x, active_region_max_y)
            # print('active region histogram =')
            # print(*histogram_grid_active_region, sep='\n')
            histogram_grid_plot.clear()
            histogram_grid_plot.matshow(
                histogram_grid_active_region, origin="lower")

        while True:
            self.step()
            if draw == True:
                # time.sleep(1)

                # 1. Replot the simulation
                obstacles_x, obstacles_y = self.path_planner.histogram_grid.get_obstacles()
                simulation_plot.scatter(
                    *self.location, color='blue')
                simulation_plot.scatter(
                    *self.target_location, color='green')
                simulation_plot.scatter(
                    obstacles_x, obstacles_y, color='red')
                active_region_min_x, active_region_min_y, active_region_max_x, active_region_max_y = self.path_planner.histogram_grid.get_active_region(
                    self.location)
                rectangle.set_bounds(active_region_min_x, active_region_min_y, active_region_max_x -
                                     active_region_min_x, active_region_max_y - active_region_min_y)

                # 2. Replot the polar histogram
                # sectors = self.path_planner.get_sectors() # NOTE: sectors are only valid
                num_bins = self.path_planner.polar_histogram.num_bins
                valley_threshold = self.path_planner.valley_threshold
                polar_histogram_by_angle = self.path_planner.polar_histogram.get_angle_certainty()
                # NOTE: instead of sectors, get polar histogram bins and filter them by valley threshold
                bin_percentages = [1.0/num_bins for angle,
                                   certainty in polar_histogram_by_angle]
                bin_certainties = [certainty for angle,
                                   certainty in polar_histogram_by_angle]
                colors = ['blue' if certainty < valley_threshold else 'red' for angle,
                          certainty in polar_histogram_by_angle]
                labels = [round(np.rad2deg(angle)) for angle, certainty in polar_histogram_by_angle]
                generator = enumerate(polar_histogram_by_angle)

                def make_autopct(bin_percentages):
                    def my_autopct(pct):
                        index, (angle, certainty) = next(generator)
                        return '{certainty:.1f}'.format(certainty=certainty)
                    return my_autopct

                polar_plot.clear()
                # polar_plot.pie(bin_percentages, colors=colors, labels=labels, startangle=0, counterclock=True, autopct=make_autopct(bin_percentages))
                polar_plot.pie(bin_percentages, colors=colors, labels=labels,
                               startangle=0, autopct=make_autopct(bin_percentages))

                # 3. Replot the histogram_grid
                histogram_grid_active_region = self.path_planner.histogram_grid.get_histogram_grid_active_region(
                    active_region_min_x, active_region_min_y, active_region_max_x, active_region_max_y)
                # print('active region histogram =')
                # print(*histogram_grid_active_region, sep='\n')
                histogram_grid_plot.clear()
                histogram_grid_plot.matshow(
                    histogram_grid_active_region, origin="lower")

                # 4. Actually display the plots
                # display.clear_output(wait=True) # NOTE: Uncomment this for animation. Comment this out if you want to see all steps.
                display.display(plt.gcf())

    def print_histogram(self):
        self.path_planner.print_histogram()

    def get_polar_bins(self):
        return self.polar_histogram.getPolarBins()


def concatenate(*lists):
    return itertools.chain(*lists)
