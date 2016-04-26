class Frontier:
    def __init__(self, gridpos_set=None):
        self.gridpos_set = gridpos_set;

    def add_grid_pos(self, gridpos):
        self.gridpos_set.add(gridpos)

    def get_center(self):
        x = [p[0] for p in self.gridpos_set]
        y = [p[1] for p in self.gridpos_set]
        center = (sum(x) / len(self.gridpos_set), sum(y) / len(self.gridpos_set))
        return center
    
    def get_distanceToFrontier(f, robotLocation):
        centerOfFrontier = get_center(f)
        return math.sqrt((centerOfFrontier.x - robotLocation.x) ** 2 + (centerOfFrontier.y - robotLocation.y) ** 2)
        
    def get_size(self):
        return len(self.gridpos_set)

    def __contains__(self, item):
        return item in self.gridpos_set
