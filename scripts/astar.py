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

    def getPoint(self, info):
        point = Point()
        resolution = info.resolution
        offsetX = info.origin.position.x
        offsetY = info.origin.position.y
        # added secondary offset
        point.x = (self.gridpos[0] * resolution) + offsetX + (1.5 * resolution)
        # added secondary offset ... Magic ?
        point.y = (self.gridpos[1] * resolution) + offsetY - (.5 * resolution)
        point.z = 0
        return point

    def getCost(self):
        return self.f_cost + self.g_cost

class AStar:
    def __init__(self, occupancygrid, start, goal):
        self.map2d = {}
        self.occupancygridinfo = occupancygrid.info

        for i in range(1, occupancygrid.info.height):  # height should be set to hieght of grid
            k = k + 1
            for j in range(1, occupancygrid.info.width):  # width should be set to width of grid
                k = k + 1
                # print k # used for debugging
                if (grid[k] < 50): # navigable cells
                    newnode = AStarNode()
    
    def getgcost(self, from, to):

