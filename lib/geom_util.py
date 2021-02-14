import numpy as np
from operator import sub


def get_angle_between_points(discrete_start, discrete_end):
    """
    Returns the angle between the line between pos2 and posRef and the horizontal along positive i direction.
    """
    # discrete_displacement = get_discrete_displacement(discrete_start, discrete_end)
    y_del = discrete_end[1] - discrete_start[1]
    x_del = discrete_end[0] - discrete_start[0]
    angle = np.arctan2(y_del, x_del)
    return angle % (2 * np.pi)  # TODO: this messed smoothing up somehow


def get_discrete_displacement(discrete_start, discrete_end):
    return tuple(map(sub, discrete_start, discrete_end))
