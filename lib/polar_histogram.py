"""
polar_histogram.py

A polar histogram means this, assuming bin_width=36
(therefore num_bins = 2*math.pi / 36 = 10):


index, corresponding_angle, histogram_angle
0, 0, 123
1, 36, 0
2, 72, 30
...
9, 324, 0

(equation: i * bin_width = angle)

However, we only keep index in a flat array for histograms, so we don't natively get/set by angle
but instead translate to and from angle.
"""
# PolarHistogram class creates an object to represent the Polar Histogram
import math
import numpy as np


class PolarHistogram:
    def __init__(self, num_bins, low_threshold, high_threshold):
        """
        Creates a Polar Histogram object with the number of bins passed.

        Args:
            num_bins: Number of bins to divide polar space around robot into.
            bin_width: Angular width around robot included in each bin.
            histogram: Array storing the values of the polar histogram.
        """
        self.num_bins = num_bins
        self.bin_width = 2*math.pi/num_bins
        self._polar_histogram = [0] * num_bins
        self.low_threshold = low_threshold
        self.high_threshold = high_threshold

        self.open_bins = []
        self.open_bins_unmasked = []
        self.l_limit, self.r_limit = 0, 0

    def wrap(self, bin_index):
        """Helper method for covering out of bounds bin_index."""
        while bin_index < 0:
            bin_index += self.num_bins

        while bin_index >= self.num_bins:
            bin_index -= self.num_bins

        return bin_index

    def get(self, bin_index):
        """custom getter covering cases where bin_index is out of bounds."""
        bin_index = self.wrap(bin_index)

        return self._polar_histogram[bin_index]

    def set(self, bin_index, value):
        """custom setter covering cases where bin_index is out of bounds."""
        bin_index = self.wrap(bin_index)

        self._polar_histogram[bin_index] = value

    def get_bin_index_from_angle(self, angle):
        """Returns index 0 <= i < nBins that corresponds to a. "Wraps" a around histogram."""
        while angle < 0:
            angle += 2*math.pi
        while angle > 2*math.pi:
            angle -= 2*math.pi

        return int(angle // self.bin_width)

    def get_middle_angle_of_bin(self, bin_index):
        """Returns the angle in the middle of the bin."""
        bin_index = self.wrap(bin_index)
        return (bin_index + 0.5) * self.bin_width

    def get_certainty_from_angle(self, angle):
        """Returns the value of the histogram for the specified bin."""
        return self.get_certainty(self.get_bin_index_from_angle(angle))

    def add_certainty_to_bin_at_angle(self, angle, delta_certainty):
        """Adds the passed value to the current value of the histogram grid."""
        bin_index = self.get_bin_index_from_angle(angle)
        self._polar_histogram[bin_index] += delta_certainty

    def smooth_histogram(self, l):
        """Smoothing function that smooths the values of the histogram using a moving average."""
        smoothed_histogram = [0] * self.num_bins
        for k_i in range(self.num_bins):

            smoothed_histogram[k_i] = sum(
                [(l - abs(k_i-l_i)) * self.get(l_i) for l_i in range(k_i-l+1, k_i+l)]) / (2*l+1)

        self._polar_histogram = smoothed_histogram

    def __str__(self):
        string = 'index, angle, certainty\n'
        for i, certainty in enumerate(self._polar_histogram):
            string += " ".join((str(i), str(i * self.bin_width), str(certainty))) + '\n'
        return string

    def get_angle_certainty(self):
        """Instead of (bin_index, certainty), return (angle, certainty) pairs."""
        return [(i * self.bin_width, certainty) for i, certainty in enumerate(self._polar_histogram)]

    def reset(self):
        self._polar_histogram = [0] * self.num_bins

    # should only be called once per timestep
    def recalculate_open_bins(self):
        result = [bin_index for bin_index, certainty
                  in enumerate(self._polar_histogram)
                  if certainty < self.low_threshold or
                  (certainty < self.high_threshold and bin_index in self.open_bins)]
        self.open_bins_unmasked = result
        self.open_bins = [bin for bin in result if is_between_angles(
            self.get_middle_angle_of_bin(bin), self.l_limit, self.r_limit)]

    def get_open_bins(self):
        return self.open_bins

    def get_open_bins_unmasked(self):
        return self.open_bins_unmasked

    def get_num_bins(self):
        return self.num_bins

    def get_sectors(self):
        self.recalculate_open_bins()
        # return early if every sector is under or over the threshold
        if self.num_bins == len(self.open_bins):
            return [(0, self.num_bins - 1)]
        elif len(self.open_bins) == 0:
            return []
        sectors = []
        last_bin = start_bin = self.open_bins[0]

        for bin in self.open_bins[1:]:
            # if a new bin is starting
            if last_bin + 1 != bin:
                sectors.append((start_bin, last_bin))
                start_bin = bin
            last_bin = bin

        if last_bin == self.num_bins - 1 and sectors and sectors[0][0] == 0:
            sectors[0] = (start_bin, sectors[0][1])
        else:
            sectors.append((start_bin, last_bin))

        return sectors

    def mask(self, coords, loc, dir, radius):
        l_limit = r_limit = dir + math.pi
        l_traj_cent = (loc[0] + radius*math.cos(dir + math.pi/2),
                       loc[1] + radius*math.sin(dir + math.pi/2))
        r_traj_cent = (loc[0] + radius*math.cos(dir - math.pi/2),
                       loc[1] + radius*math.sin(dir - math.pi/2))
        sqr_radius = radius ** 2
        # print("dir:", np.degrees(dir))
        print("loc:", loc)
        for coord in coords:
            beta = angle_two_points(loc, coord)
            # print(np.degrees(beta))
            # print(math.sqrt(sqr_dist(l_traj_cent, coord)))
            if is_between_angles(beta, l_limit, dir) and sqr_dist(l_traj_cent, coord) < sqr_radius:
                l_limit = beta
            if is_between_angles(beta, dir, r_limit) and sqr_dist(r_traj_cent, coord) < sqr_radius:
                r_limit = beta

        self.l_limit, self.r_limit = l_limit, r_limit
        print(l_limit, r_limit)


# is_between_angles returns true if query is hit when you move clockwise from start to end
# if any two of start, end, or query equal each other, should always be true
def is_between_angles(query, start, end, degrees=False):
    if degrees:
        query = np.radians(query)
        start = np.radians(start)
        end = np.radians(end)
    new_end = end - start + 2*np.pi if end - start < 0 else end - start
    if np.abs(new_end % (2*np.pi) - np.pi) < 1e-7:
        # if start and end are almost in opposite directions,
        # any query should return True
        return True
    new_query = query - start + 2*np.pi if (query - start) <= 0 else query - start
    return new_query >= new_end


# returns angle from "from_p" in order to reach "to_p"
def angle_two_points(from_p, to_p):
    return math.atan2(to_p[1] - from_p[1], to_p[0] - from_p[0])


def sqr_dist(p1, p2):
    return (p2[0] - p1[0])**2 + (p2[1] - p1[1])**2


assert(is_between_angles(0, 1, 359, degrees=True))
assert(is_between_angles(270, 350, 180, degrees=True))
assert(is_between_angles(270, 0, 180, degrees=True))
assert(is_between_angles(-90, 0, 180, degrees=True))
assert(is_between_angles(-90, 0, -180, degrees=True))

'''
// C code adapted from https://math.stackexchange.com/questions/1044905/simple-angle-between-two-angles-of-circle
#include <stdio.h>

bool isBetween(float start, float end, float mid) {     
    end = (end - start) < 0.0f ? end - start + 360.0f : end - start;    
    mid = (mid - start) <= 0.0f ? mid - start + 360.0f : mid - start; 
    return (mid >= end); 
}

int main() {
    // Write C code here
    printf("Should be true\n");
    fputs(isBetween(270, 90, 180) ? "true\n" : "false\n", stdout);
    fputs(isBetween(0, 358, 359) ? "true\n" : "false\n", stdout);
    fputs(isBetween(1, 359, 0) ? "true\n" : "false\n", stdout);
    fputs(isBetween(0, 180, 270) ? "true\n" : "false\n", stdout);
    fputs(isBetween(0, 0, 270) ? "true\n" : "false\n", stdout);
    fputs(isBetween(0, 270, 0) ? "true\n" : "false\n", stdout);
    fputs(isBetween(270, 0, 0) ? "true\n" : "false\n", stdout);
    fputs(isBetween(0, 0, 0) ? "true\n" : "false\n", stdout);
    printf("Should be false\n");
    fputs(isBetween(310, 20, 0) ? "true\n" : "false\n", stdout);
    fputs(isBetween(270, 90, 0) ? "true\n" : "false\n", stdout);
    fputs(isBetween(0, 359, 1) ? "true\n" : "false\n", stdout);
    fputs(isBetween(1, 0, 359) ? "true\n" : "false\n", stdout);
    
    return 0;
}
'''
