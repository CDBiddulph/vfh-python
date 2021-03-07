"""Used to plot nav simulation. Initial bikestate and waypoints can be changed
at the bottom of the file"""

import sys
import math

from numpy.lib.arraypad import pad
from bike import Bike
from histogram_grid import HistogramGrid
from polar_histogram import PolarHistogram
from pure_pursuit import PPPathPlanner
from vfh_path_planner import VFHPathPlanner
from combined_path_planner import CombinedPathPlanner

import numpy as np

# Import graphics library
import matplotlib

from matplotlib import pyplot as plt
from matplotlib import animation
from matplotlib import collections as mc
from matplotlib.path import Path
from matplotlib.patches import Wedge, PathPatch, Circle

# Use a different graphics backend and function for OS X
if sys.platform == "darwin":
    matplotlib.use("TkAgg")
    print("Using the TkAgg backend because OS X. Animation speed might be slower. /n \
    Also, has not been tested with the addition of vfh_star")
    def get_loop_function(): return loop_matplotlib
else:
    def get_loop_function(): return loop_matplotlib_blitting

ANIM_INTERVAL = 1


def loop_matplotlib(bike):
    """This function uses adding and removing patches to animate the bike."""
    plt.ion()  # enables interactive plotting

    fig = plt.figure()

    if hasattr(bike, 'waypoints'):
        ax = plt.axes(**find_display_bounds(bike.waypoints))
    else:
        ax = plt.axes({'xlim': [-10, 10], 'ylim': [-10, 10]})

    if hasattr(bike, 'paths'):
        paths = bike.paths
        lc = mc.LineCollection(paths, linewidths=2, color="blue")
        ax.add_collection(lc)

    plt.show()

    # For plotting the bicycle
    axes = plt.gca()

    # Holds past locations of the bike, for plotting
    bike_trajectory = [bike.pos]

    # We need to keep this around to clear it after path updates
    path_patch = None

    prev_bike_patch = None
    prev_lookahead_patch = None

    # Main animation loop
    while True:

        if path_patch:
            path_patch.remove()
        path_patch = PathPatch(Path(bike_trajectory), fill=False,
                               linewidth=2)
        axes.add_patch(path_patch)

        # Plot the bike as a wedge pointing in the direction bike.psi
        if prev_bike_patch:
            prev_bike_patch.remove()
        bike_heading = bike.heading * (180 / math.pi)  # Converted to degrees
        wedge_angle = 45  # The angle covered by the wedge
        bike_polygon = Wedge(bike.pos, 0.3,
                             bike_heading - wedge_angle / 2 + 180,
                             bike_heading + wedge_angle / 2 + 180, fc="black")
        axes.add_patch(bike_polygon)
        prev_bike_patch = bike_polygon
        fig.canvas.flush_events()

        bike_trajectory.append(bike.pos)

        speed, yaw_dot = bike.get_nav_command()
        bike.step(speed, yaw_dot)
    plt.ioff()
    plt.show()


def loop_matplotlib_blitting(bike, blitting=True):
    """This code uses blitting and callbacks to simulate the
    bike. Because so much of the code is shared, this function, when
    provided with the filename argument, will save video to the
    specified filename instead of displaying the animation in a
    window."""
    figure = plt.figure()
    # figure, (simulation_plot, polar_plot, histogram_grid_plot) = plt.subplots(
    #     1, 3, figsize=(18, 6))

    if hasattr(bike, 'waypoints'):
        axes = plt.axes(**find_display_bounds(bike.waypoints))
    else:
        right_bound, upper_bound = bike.path_planner.get_histogram_grid().get_real_size()
        padding = 10
        axes = plt.axes(xlim=[-padding, right_bound + padding], ylim=[-padding, upper_bound + padding])

    # Square aspect ratio for the axes
    axes.set_aspect("equal")

    # Plot paths
    lc = mc.LineCollection(bike.get_paths(), linewidths=2, color="blue")
    axes.add_collection(lc)

    # Plot obstacles
    for x, y, prob in bike.get_obstacles():
        axes.add_artist(Circle((x, y), radius=0.5, alpha=prob/100))

    # Paths and obstacles won't change, so capture them
    figure.canvas.draw()

    background = [figure.canvas.copy_from_bbox(axes.bbox)]

    # Create bike polygon
    bike_heading = bike.heading * (180 / math.pi)  # heading is psi, but in degrees
    wedge_angle = 45  # The angle covered by the wedge (degrees)
    theta1 = bike_heading - wedge_angle / 2 + 180
    theta2 = bike_heading + wedge_angle / 2 + 180
    bike_polygon = Wedge(bike.pos, 1, theta1, theta2, fc="black")
    bike_polygon.set_zorder(10)
    axes.add_artist(bike_polygon)

    # Create pure pursuit lookahead point
    pp_lookahead_polygon = Circle((0, 0), radius=0.5, color="r")
    pp_lookahead_polygon.set_zorder(9)
    axes.add_artist(pp_lookahead_polygon)

    # Create lookahead point from low-level direction tracking
    dir_lookahead_polygon = Circle((0, 0), radius=0.5, color="orange")
    dir_lookahead_polygon.set_zorder(9)
    axes.add_artist(dir_lookahead_polygon)

    # Create bike trajectory
    bike_trajectory_polygon = axes.plot([0, 0], [0, 0], "g")[0]

    # Set up trajectory data
    bike_traj_x = [bike.pos[0]]  # Just the x-coords
    bike_traj_y = [bike.pos[1]]  # Just the y-coords
    add_traj_x = bike_traj_x.append
    add_traj_y = bike_traj_y.append

    # Set up resizing handlers
    listener_id = [None]

    def safe_draw():
        canvas = figure.canvas
        if listener_id[0]:
            canvas.mpl_disconnect(listener_id[0])
        canvas.draw()
        listener_id[0] = canvas.mpl_connect("draw_event", grab_background)

    def grab_background(event=None):
        transient_polygons = (bike_polygon, pp_lookahead_polygon, dir_lookahead_polygon)
        for polygon in transient_polygons:
            polygon.set_visible(False)
        safe_draw()
        background[0] = figure.canvas.copy_from_bbox(figure.bbox)
        for polygon in transient_polygons:
            polygon.set_visible(True)
        blit()

    def blit():
        figure.canvas.restore_region(background[0])
        axes.draw_artist(bike_polygon)
        axes.draw_artist(pp_lookahead_polygon)
        axes.draw_artist(dir_lookahead_polygon)
        figure.canvas.blit(axes.bbox)

    listener_id[0] = figure.canvas.mpl_connect("draw_event", grab_background)

    # This timer runs simulation steps and draws the results
    figure_restore = figure.canvas.restore_region
    figure_blit = figure.canvas.blit

    def full_step(data=None):
        figure_restore(background[0])
        bike.step(*bike.get_nav_command())

        # Update bike polygon properties and redraw it
        wedge_dir = bike.heading * (180 / math.pi) + 180
        bike_polygon.set(center=bike.pos,
                         theta1=wedge_dir - wedge_angle / 2,
                         theta2=wedge_dir + wedge_angle / 2)
        axes.draw_artist(bike_polygon)

        # Update PP lookahead polygon properties and redraw it
        pp_lookahead_polygon.set(center=bike.get_nav_lookahead())
        axes.draw_artist(pp_lookahead_polygon)

        # Update PP lookahead polygon properties and redraw it
        dir_lookahead_polygon.set(center=bike.get_dir_lookahead())
        axes.draw_artist(dir_lookahead_polygon)

        # Update trajectory and redraw it
        add_traj_x(bike.pos[0])
        add_traj_y(bike.pos[1])
        bike_trajectory_polygon.set_xdata(bike_traj_x)
        bike_trajectory_polygon.set_ydata(bike_traj_y)
        axes.draw_artist(bike_trajectory_polygon)

        # Redraw bike
        figure_blit(axes.bbox)

        # Replot polar histogram
        # plot_polar_histogram()

        # Start the update & refresh timer
    if blitting:
        figure.canvas.new_timer(interval=ANIM_INTERVAL, callbacks=[(full_step, [], {})]).start()
    else:
        ani = animation.FuncAnimation(figure, full_step, frames=xrange(0, 20000))

    # Display the window with the simulation
    plt.show()


def plot_polar_histogram():
    polar_histogram = bike.get_path_planner().get_polar_histogram()
    num_bins = polar_histogram.num_bins
    valley_threshold = vfh_path_planner.valley_threshold
    polar_histogram_by_angle = polar_histogram.get_angle_certainty()
    # NOTE: instead of sectors, get polar histogram bins and filter them by valley threshold
    bin_percentages = [1.0/num_bins for _ in range(len(polar_histogram_by_angle))]
    colors = ['blue' if certainty < valley_threshold else 'red' for _,
              certainty in polar_histogram_by_angle]
    labels = [round(np.rad2deg(angle)) for angle, _ in polar_histogram_by_angle]
    generator = enumerate(polar_histogram_by_angle)

    def make_autopct(bin_percentages):
        def my_autopct(pct):
            index, (angle, certainty) = next(generator)
            return '{certainty:.1f}'.format(certainty=certainty)
        return my_autopct

    plt.pie(bin_percentages, colors=colors, labels=labels,
            startangle=0, autopct=make_autopct(bin_percentages))


def find_display_bounds(waypoints):
    """Given a set of waypoints, return {xlim, ylim} that can fit them."""
    xlim = [float("inf"), -float("inf")]  # min, max
    ylim = [float("inf"), -float("inf")]  # min, max
    padding = 5
    for waypoint in waypoints:
        if waypoint[0] < xlim[0]:
            xlim[0] = waypoint[0]
        elif waypoint[0] > xlim[1]:
            xlim[1] = waypoint[0]

        if waypoint[1] < ylim[0]:
            ylim[0] = waypoint[1]
        elif waypoint[1] > ylim[1]:
            ylim[1] = waypoint[1]
    xlim, ylim = (xlim[0] - padding, xlim[1] + padding), (ylim[0] - padding, ylim[1] + padding)
    return {"xlim": xlim, "ylim": ylim}


if __name__ == '__main__':
    STARTING_LOC = (0, 0)
    STARTING_HEADING = 0
    TARGET_LOC = (50, 50)

    histogram_grid = HistogramGrid.from_png_map("maps/map1_s.png", 16, 1)
    polar_histogram = PolarHistogram(36)

    waypoints = [(0, 0), (40, 15), (30, 50), (50, 50)]
    pp_path_planner = PPPathPlanner(waypoints, lookahead_dist=10)
    vfh_path_planner = VFHPathPlanner(histogram_grid, polar_histogram, valley_threshold=10000)
    combined_path_planner = CombinedPathPlanner(pp_path_planner, vfh_path_planner)
    bike = Bike(combined_path_planner, STARTING_LOC, TARGET_LOC, STARTING_HEADING, dir_lookahead_dist=5)

    get_loop_function()(bike)
