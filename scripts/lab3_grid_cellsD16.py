#!/usr/bin/env python

import rospy
from nav_msgs.msg import GridCells
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from kobuki_msgs.msg import BumperEvent
import tf
import numpy
import math
import rospy
import tf
import numpy
import math
from astar import AStarNode
from move import Movement
import copy


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

# goal callback
def readGoal(goal):
    global start_pose
    global goal_pose
    goal_pose = goal.pose
    aStar(start_pose, goal_pose)
    print goal_pose
    # Start Astar

# start callback
def readStart(startPos):
    global start_pose
    print 'start pose received'
    start_pose = startPos.pose.pose
    print startPos.pose


def aStar(start, goal):
    global navigable_gridpos
    global pubpath
    global pubopen
    global pubclose
    openNodes = {}
    closedNodes = {}

    goalgridpos = pose2gridpos(goal)
    startgridpos = pose2gridpos(start)

    print goalgridpos
    print startgridpos

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
            publishPoints(pubpath, path)
            publishWaypoints(path)
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
        publishPoints(pubopen, openNodes.keys())
        publishPoints(pubclose, closedNodes.keys())

    raise Exception('no path found')


# publishes map to rviz using gridcells type
def getNeighbors(me_gridpos, navigable_gridpos):
    assert me_gridpos in navigable_gridpos
    offset = [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1)]
    neighbor_pos = []
    for o in offset:
        n_pos = (me_gridpos[0] + o[0], me_gridpos[1] + o[1])
        if n_pos in navigable_gridpos:
            neighbor_pos.append(n_pos)
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

# convert from pose to position in grid
def pose2gridpos(pose):
    global resolution
    global offsetX
    global offsetY
    gridx = int((pose.position.x - offsetX - (.5 * resolution)) / resolution)
    # added secondary offset ... Magic ?
    gridy = int((pose.position.y - offsetY - (.5 * resolution)) / resolution)
    return (gridx, gridy)

# useless
def publishCells(grid):

#begin test:
    testGrid = grid
 #   grid = expandObsticals(testGrid)
    
#end test:


    
    global pub
    row = ""
    global navigable_gridpos

    navigable_gridpos = []
    unnavigable_gridpos = []

    # resolution and offset of the map
    k = 0
    cells = GridCells()
    cells.header.frame_id = 'map'
    cells.cell_width = resolution
    cells.cell_height = resolution

    for i in range(0, height):  # height should be set to hieght of grid
        for j in range(0, width):  # width should be set to width of grid
            #print grid[] # used for debugging
            
            if (grid[k] < 50):
                row = row + "    "
                navigable_gridpos.append((j, i))
            if (grid[k] == 100):
                unnavigable_gridpos.append((j, i))
                if(i > 1):
                    point = getPoint((j, i-1))
                    cells.cells.append(point)
                    unnavigable_gridpos.append(point)
                    
                if(j > 1):
                    point = getPoint((j-1, i))
                    cells.cells.append(point)
                    unnavigable_gridpos.append(point)
                    
                if(width > j):
                    point = getPoint((j+1, i))
                    cells.cells.append(point)
                    unnavigable_gridpos.append(point)
                if(height > i):
                    point = getPoint((j, i+1))
                    cells.cells.append(point)
                    unnavigable_gridpos.append(point)
                
                
                
                
                
                
                
                
                
                
                
                
#                row = row + " " + str (grid[k]);
#                point = getPoint((j, i))
#                cells.cells.append(point)
            k = k + 1
        print "\n"+ row;
        row = ""
    pub.publish(cells)
    
    
    
    
def expandObsticals(grid):
    row = ""
    global navigable_gridpos
    
    
    print len(grid)

    navigable_gridpos = []
    unnavigable_gridpos = []

    # resolution and offset of the map
    k = 0
    cells = GridCells()
    cells.header.frame_id = 'map'
    cells.cell_width = resolution
    cells.cell_height = resolution

    for i in range(0, height):  # height should be set to hieght of grid
        for j in range(0, width):  # width should be set to width of grid
            #print grid[] # used for debugging
            
            
            if (grid[k] < 50):
                navigable_gridpos.append((j, i))
            if (grid[k] == 100):
                unnavigable_gridpos.append((j, i))
                if(i > 1):
                    point = getPoint((j, i-1))
                    cells.cells.append(point)
                    unnavigable_gridpos.append(point)
                    
                if(j > 1):
                    point = getPoint((j-1, i))
                    cells.cells.append(point)
                    unnavigable_gridpos.append(point)
                    
                if(width > j):
                    point = getPoint((j+1, i))
                    cells.cells.append(point)
                    unnavigable_gridpos.append(point)
                if(height > i):
                    point = getPoint((j, i+1))
                    cells.cells.append(point)
                    unnavigable_gridpos.append(point)
                    
            k = k + 1
    print unnavigable_gridpos
    return unnavigable_gridpos
    




# publish a gridcell to pub using points in a list of (x,y) gridpos
def publishPoints(pub, listofgridpos):
    global width
    global height
    global resolution
    global offsetX
    global offsetY

    cells = GridCells()
    cells.header.frame_id = 'map'
    cells.cell_width = resolution
    cells.cell_height = resolution

    for g in listofgridpos:
        point = getPoint(g)
        cells.cells.append(point)

    pub.publish(cells)


# convert from grid position (x,y) to real world coordinate. return a Point.
def getPoint(gridpos):
    global width
    global height
    global resolution
    global offsetX
    global offsetY

    point = Point()
    # added secondary offset
    point.x = (gridpos[0] * resolution) + offsetX + (.5 * resolution)
    # added secondary offset ... Magic ?
    point.y = (gridpos[1] * resolution) + offsetY + (.5 * resolution)
    point.z = 0

    return point


def getDirection(fr, to):
    dx = to[0] - fr[0]
    dy = to[1] - fr[1]
    return math.atan2(dy, dx)
    
#def expandObsticals(inputMap)
        

# generate a Path from a list of gridpos (x,y)
def publishWaypoints(list_of_gridpos):
    global goal_pose
    global pubrealpath
    wp = Path()
    wp.header.frame_id = 'map'
    poses = []
    lastdirection = -999
    for i in range(0, len(list_of_gridpos) - 1):
        direction = getDirection(list_of_gridpos[i], list_of_gridpos[i + 1])
        if direction != lastdirection:
            lastdirection = copy.deepcopy(direction)
            newPose = PoseStamped()
            newPose.header.frame_id = 'map'
            newPose.pose = Pose()
            newPose.pose.position = getPoint(list_of_gridpos[i])
            quaternion = tf.transformations.quaternion_from_euler(0, 0, direction)
            newPose.pose.orientation.x = quaternion[0]
            newPose.pose.orientation.y = quaternion[1]
            newPose.pose.orientation.z = quaternion[2]
            newPose.pose.orientation.w = quaternion[3]
            poses.append(newPose)
    newPose = PoseStamped()
    newPose.header.frame_id = 'map'
    newPose.pose = goal_pose
    poses.append(newPose)
    wp.poses = poses
    pubrealpath.publish(wp)
    return wp

def navToWaypoints(list_of_gridpos):
    waypoints = getWaypoints(list_of_gridpos)
    
    count = 0
    while(len(waypoints) < count):
        navToPose(waypoints[count].pose)
        count = count + 1

def run():
    global pub
    global pubpath
    global pubopen
    global pubclose
    global pubrealpath
    rospy.init_node('lab3')
    sub = rospy.Subscriber("/map", OccupancyGrid, mapCallBack)
    pub = rospy.Publisher("/map_check", GridCells, queue_size=1)
    # you can use other types if desired
    pubpath = rospy.Publisher("/path", GridCells, queue_size=1)
    pubopen = rospy.Publisher("/opennodes", GridCells, queue_size=1)
    pubclose = rospy.Publisher("/closednodes", GridCells, queue_size=1)
    pubway = rospy.Publisher("/waypoints", GridCells, queue_size=1)
    pubrealpath = rospy.Publisher("/realpath", Path, queue_size=1)
    # change topic for best results
    goal_sub = rospy.Subscriber('move_base_simple/goalrbe', PoseStamped, readGoal, queue_size=1)
    # change topic for best results
    goal_sub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, readStart, queue_size=1)

    # wait a second for publisher, subscribers, and TF
    rospy.sleep(1)

    print "Running Lab 3 Code..."

    while (1 and not rospy.is_shutdown()):
        publishCells(mapData)  # publishing map data every 2 seconds
        rospy.sleep(2)


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
