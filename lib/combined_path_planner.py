class CombinedPathPlanner():
    def __init__(self, pp_path_planner, vfh_path_planner):
        self.pp = pp_path_planner
        self.vfh = vfh_path_planner

    def get_best_angle(self, robot_loc):
        self.pp.update_lookahead(robot_loc)
        return self.vfh.get_best_angle(robot_loc, self.pp.get_lookahead())

    def get_histogram_grid(self):
        return self.vfh.get_histogram_grid()

    def get_polar_histogram(self):
        return self.vfh.get_polar_histogram()

    def get_paths(self):
        return self.pp.paths

    def get_lookahead(self):
        return self.pp.get_lookahead()
