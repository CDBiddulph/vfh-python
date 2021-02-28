import math

# Used for the simulation
SMALL_DISTANCE = 0.01
SMALL_ANGLE = 0.01


class PPPathPlanner:
    def __init__(self, waypoints, lookahead_dist=10, skip_limit=2):
        """
        x = starting x-coordinate, in m (+x points to the right)
        y = starting y-coordinate, in m (+y points upwards)
        heading = starting header, in radians (0 is +x axis, increases counterclockwise)
        waypoints = same format as the bike format - list of waypoints
                    e.g. the path from 0,0 to 0,5 then to 7,6 would be passed in like this:
                [(0, 0), (0, 5), (7, 6)]
        """
        self.waypoints = waypoints
        self.paths = init_paths(waypoints)
        self.lookahead_index = 0
        self.lookahead_point = None
        # how many path segments can be skipped at once
        self.skip_limit = 2

        self.LOOKAHEAD_DIST = lookahead_dist

    def waypoint_distances(self):
        output = []
        if len(self.waypoints) > 0:
            output.append(0)  # starts with 0 for self.waypoints[0]
        for p in self.paths:
            output.append(output[-1] + math.sqrt((p[1][0] - p[0][0]) ** 2 + (p[1][1] - p[0][1]) ** 2))
        return output

    def update_lookahead(self, robot_loc):
        path_i = int(self.lookahead_index)
        intersect = None
        # first argument of min ensures no significant skipping on the path
        while path_i < min(int(self.lookahead_index) + self.skip_limit, len(self.paths)):
            intersect = self.path_intersect(robot_loc, self.paths[path_i])
            if intersect is not None:
                # if it'll end up greater than the old lookahead_index, then add the current path_i to intersect.
                # otherwise, leave it as None, as if there were no intersect found at all.
                intersect = intersect + path_i
                if intersect > self.lookahead_index:
                    break
                else:
                    intersect = None
            path_i += 1
        if intersect is not None:
            self.lookahead_index = intersect
        # else, leave lookahead_index the same as before

        # return the point representing the point the robot should follow
        self.lookahead_point = self.lookahead_from_index()

    def lookahead_from_index(self):
        path_i = int(self.lookahead_index)
        path = self.paths[path_i]
        path_fraction = self.lookahead_index - path_i
        dx = path_fraction * (path[1][0] - path[0][0])
        dy = path_fraction * (path[1][1] - path[0][1])
        return path[0][0] + dx, path[0][1] + dy

    def path_intersect(self, robot_loc, path):
        """path is in the form ((x1, y1), (x2, y2))"""
        # d is the direction vector of path: end minus start
        d_x = path[1][0] - path[0][0]
        d_y = path[1][1] - path[0][1]
        # f is the start of path minus self coordinates
        f_x = path[0][0] - robot_loc[0]
        f_y = path[0][1] - robot_loc[1]

        a = d_x * d_x + d_y * d_y
        b = 2 * (d_x * f_x + d_y * f_y)
        c = f_x * f_x + f_y * f_y - self.LOOKAHEAD_DIST ** 2
        discriminant = b*b - 4*a*c

        if discriminant < 0:
            return None
        discriminant = math.sqrt(discriminant)
        t1 = (-b - discriminant) / (2*a)
        t2 = (-b + discriminant) / (2*a)
        if 0 <= t2 <= 1:
            return t2
        if 0 <= t1 <= 1:
            return t1
        return None

    def get_lookahead(self):
        return self.lookahead_point


def init_paths(waypoints):
    """ Initializes paths from input waypoints """
    paths = []
    if len(waypoints) < 2:
        return paths
    else:
        for i in range(1, len(waypoints)):
            paths.append((waypoints[i - 1], waypoints[i]))
        return paths
