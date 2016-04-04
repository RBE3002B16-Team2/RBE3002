#!/usr/bin/env python

import rospy
from nav_msgs.msg import GridCells
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from kobuki_msgs.msg import BumperEvent
import tf
import numpy
import math
import rospy
import tf
import numpy
import math
from astar import AStarNode


# reads in global map
def mapCallBack(data):
    global mapData
    global width
    global height
    global mapgrid
    global resolution
    global offsetX
    global offsetY
    mapgrid = data
    resolution = data.info.resolution
    mapData = data.data
    width = data.info.width
    height = data.info.height
    offsetX = data.info.origin.position.x
    offsetY = data.info.origin.position.y
    print data.info


def readGoal(goal):
    global start_pose
    goal_pose = goal.pose
    aStar(start_pose, goal_pose)
    print goal.pose
    # Start Astar


def readStart(startPos):
    global start_pose
    start_pose = startPos.pose
    print startPos.pose.pose


def aStar(start, goal):
    global navigable_gridpos
    global pubpath
    global pubopen
    global pubclose
    openNodes = {}
    closedNodes = {}

    goalgridpos = pose2gridpos(goal)
    startgridpos = pose2gridpos(start)

    # dump in the first node
    openNodes[pose2gridpos(start)] = AStarNode(
        None, 0, gethcost(startgridpos, goalgridpos), startgridpos)

    while len(openNodes) > 0:
        # get the node with lowest cost
        current = next(openNodes.iteritems())
        for i in openNodes.iteritems():
            if i[1].getCost() < current[1].getCost():
                current = i
        if current[0] == goalgridpos:
            path = getPath(current[1])
            publishPoints(pubpath,path)
            return path

        openNodes.pop(current[0])
        closedNodes[current[0]] = current[1]

        for n in getNeighbors(current[0], navigable_gridpos):  # TODO
            newNode = AStarNode(
                current, getgcost(current[0], n), gethcost(n, goalgridpos), n)
            if n in closedNodes:
                continue
            if n not in openNodes or openNodes[n].g_cost < newNode.g_cost:
                openNodes[n.gridpos] = newNode
    raise Exception('no path found')

    # create a new instance of the map

    # generate a path to the start and end goals by searching through the
    # neighbors, refer to aStar_explanied.py

    # for each node in the path, process the nodes to generate GridCells and
    # Path messages

    # Publish points


# publishes map to rviz using gridcells type
def getNeighbors(me_gridpos, navigable_gridpos):
    assert me_gridpos in navigable_gridpos
    offset = [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1)]
    neighbor_pos = []
    for o in offset:
        n_pos = (me_gridpos[0] + o[0], me_gridpos[1] + o[1])
        if n_pos in navigable_gridpos:
            neighbor_pos.append(n_pos)


def gethcost(fr, to):
    math.abs(to[0] - fr[0]) + math.abs(to[1] - fr[1])


def getgcost(fr, to):
    math.sqrt((to[0] - fr[0]) ** 2 + (to[1] - fr[1]) ** 2)


def getPath(end_node):
    path = []
    if end_node.parent is not None:
        path = getPath(end_node.parent)
    path.append(end_node.gridpos)
    return path


def pose2gridpos(pose):
    global resolution
    global offsetX
    global offsetY
    gridx = int((pose.position.x - offsetX - (1.5 * resolution)) / resolution)
    # added secondary offset ... Magic ?
    gridy = int((pose.position.y - offsetY + (.5 * resolution)) / resolution)
    return (gridx, gridy)


def publishCells(grid):
    global pub
    global navigable_gridpos
    print "publishing"

    navigable_gridpos=[]

    # resolution and offset of the map
    k = 0
    cells = GridCells()
    cells.header.frame_id = 'map'
    cells.cell_width = resolution
    cells.cell_height = resolution

    for i in range(1, height):  # height should be set to hieght of grid
        k = k + 1
        for j in range(1, width):  # width should be set to width of grid
            k = k + 1
            # print k # used for debugging
            if (grid[k] < 50):
                navigable_gridpos.append((j,i))
            if (grid[k] == 100):
                point = Point()
                # added secondary offset
                point.x = (j * resolution) + offsetX + (1.5 * resolution)
                # added secondary offset ... Magic ?
                point.y = (i * resolution) + offsetY - (.5 * resolution)
                point.z = 0
                cells.cells.append(point)
    pub.publish(cells)


def publishPoints(pub,listofgridpos):
    global width
    global height
    global resolution
    global offsetX
    global offsetY

    print "publishing"
    # resolution and offset of the map
    k = 0
    cells = GridCells()
    cells.header.frame_id = 'map'
    cells.cell_width = resolution
    cells.cell_height = resolution

    for g in listofgridpos:
        point = Point()
        # added secondary offset
        point.x = (g[0] * resolution) + offsetX + (1.5 * resolution)
        # added secondary offset ... Magic ?
        point.y = (g[1] * resolution) + offsetY - (.5 * resolution)
        point.z = 0
        cells.cells.append(point)

    pub.publish(cells)
# Main handler of the project


def run():
    global pub
    global pubpath
    global pubopen
    global pubclose
    rospy.init_node('lab3')
    sub = rospy.Subscriber("/map", OccupancyGrid, mapCallBack)
    pub = rospy.Publisher("/map_check", GridCells, queue_size=1)
    # you can use other types if desired
    pubpath = rospy.Publisher("/path", GridCells, queue_size=1)
    pubopen = rospy.Publisher("/opennodes", GridCells, queue_size=1)
    pubclose = rospy.Publisher("/closednodes", GridCells, queue_size=1)
    pubway = rospy.Publisher("/waypoints", GridCells, queue_size=1)
    # change topic for best results
    goal_sub = rospy.Subscriber(
        'move_base_simple/goalrbe', PoseStamped, readGoal, queue_size=1)
    # change topic for best results
    goal_sub = rospy.Subscriber(
        'initialpose', PoseWithCovarianceStamped, readStart, queue_size=1)

    # wait a second for publisher, subscribers, and TF
    rospy.sleep(1)

    while (1 and not rospy.is_shutdown()):
        publishCells(mapData)  # publishing map data every 2 seconds
        rospy.sleep(2)
        print("Complete")


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
