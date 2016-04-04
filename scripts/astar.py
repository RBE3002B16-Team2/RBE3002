from geometry_msgs.msg import Point


class AStarNode:
    def __init__(self, parent, d_g, h, gridpos):
        self.h_cost = h
        self.parent = parent
        self.gridpos = gridpos
        if parent is None:
            self.g_cost = d_g
        else:
            self.g_cost = d_g + parent.g_cost

    def getCost(self):
        return self.h_cost + self.g_cost



