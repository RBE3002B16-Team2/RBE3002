import itertools
from geometry_msgs.msg import Point
import math


def aStar(navigable_gridpos, startgridpos, goalgridpos, nodes_callback=None):
    if goalgridpos not in navigable_gridpos:
        print 'no path found because goal not navigable'
        raise NoPathFoundException

    openNodes = {}
    closedNodes = {}

    # dump in the first node
    openNodes[startgridpos] = AStarNode(
        None, 0, gethcost(startgridpos, goalgridpos), startgridpos)

    while len(openNodes) > 0:
        # get the node with lowest cost
        current = next(openNodes.iteritems())
        for i in openNodes.iteritems():
            if i[1].getCost() < current[1].getCost():
                current = i

        if current[0] == goalgridpos:
            path = getPath(current[1])
            nodes_callback(openNodes.keys(), closedNodes.keys(), path)
            return path

        openNodes.pop(current[0])
        closedNodes[current[0]] = current[1]

        for n in getNeighbors(current[0], navigable_gridpos):
            newNode = AStarNode(
                current[1], getgcost(current[0], n), gethcost(n, goalgridpos), n)
            if n in closedNodes:
                continue
            if n not in openNodes or openNodes[n].g_cost > newNode.g_cost:
                openNodes[n] = newNode

        try:
            pass
            # nodes_callback(openNodes.keys(),closedNodes.keys(),[])
        except:
            pass

    raise NoPathFoundException()


class NoPathFoundException(Exception):
    pass


# publishes map to rviz using gridcells type
def getNeighbors(me_gridpos, navigable_gridpos=None):
    # assert me_gridpos in navigable_gridpos
    offset = [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1)]
    neighbor_pos = set()
    for o in offset:
        n_pos = (me_gridpos[0] + o[0], me_gridpos[1] + o[1])
        if navigable_gridpos is None:
            neighbor_pos.add(n_pos)
        elif n_pos in navigable_gridpos:
            neighbor_pos.add(n_pos)
    return neighbor_pos

def getNeighborsByRadius(me_gridpos, radius, including_me=False):
    # assert me_gridpos in navigable_gridpos
    offset_x = range(-radius,radius+1)
    offset_prod = itertools.product(offset_x,offset_x)

    neighbor_pos = set()
    for o in offset_prod:
        if o[0]**2 + o[1]**2 <= radius**2:
            n_pos = (me_gridpos[0] + o[0], me_gridpos[1] + o[1])
            if not including_me and o[0] == 0 and o[1] == 0:
                continue
            neighbor_pos.add(n_pos)
    return neighbor_pos

# fr and to are tuples (x,y)
def gethcost(fr, to):
    return abs(to[0] - fr[0]) + abs(to[1] - fr[1])


# fr and to are tuples (x,y)
def getgcost(fr, to):
    return math.sqrt((to[0] - fr[0]) ** 2 + (to[1] - fr[1]) ** 2)


# end_node: type of AStarNode. returns list of tuples (x,y) in gridpos
def getPath(end_node):
    path = []
    if end_node.parent is not None:
        path = getPath(end_node.parent)
    path.append(end_node.gridpos)
    return path


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
