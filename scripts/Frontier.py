# Frontier.py


class Frontier:
    def __init__(self, gridpos_set=None):
        self.gridpos_set = gridpos_set;

    def add_grid_pos(self, gridpos):
        self.gridpos_set.add(gridpos)

    def get_center(self):
        from FrontierExplorer import find_closest_gridpos

        x = [p[0] for p in self.gridpos_set]
        y = [p[1] for p in self.gridpos_set]
        center = (sum(x) / len(self.gridpos_set), sum(y) / len(self.gridpos_set))
        center = (int(center[0]), int(center[1]))
        center_on_me, _ = find_closest_gridpos(center, self.gridpos_set)
        return center_on_me

    def get_size(self):
        return len(self.gridpos_set)

    def __len__(self):
        return self.get_size()

    def __contains__(self, item):
        return item in self.gridpos_set
