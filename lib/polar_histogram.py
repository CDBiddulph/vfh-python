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
        # print("polar_histogram: adding certainty %s to bin #%s = %s" % (delta_certainty, bin_index, angle))
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
                  if certainty < self.low_threshold]  # or
        #   (certainty < self.high_threshold and bin_index in self.open_bins)]
        self.open_bins = result
        return result

    # like recalculate_open_bins, but doesn't recalculate or depend on the previous results,
    # can be called multiple times per timestep
    def get_open_bins(self):
        return self.open_bins

    def get_num_bins(self):
        return self.num_bins

    def get_sectors(self):
        open_bins = self.get_open_bins()
        num_bins = self.num_bins
        # return early if every sector is under or over the threshold
        if num_bins == len(open_bins):
            return [(0, num_bins - 1)]
        elif len(open_bins) == 0:
            return []
        sectors = []
        last_bin = start_bin = open_bins[0]

        for bin in open_bins[1:]:
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
