#!/usr/bin/env python

import rospy
from nav_msgs.msg import GridCells
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path
import tf
import numpy
import math


def cost_map_to_navigable_grid_cell(og):
    resolution = og.info.resolution
    width = og.info.width
    height = og.info.height
    offsetX = og.info.origin.position.x
    offsetY = og.info.origin.position.y

    # resolution and offset of the map
    k = 0
    cells = GridCells()
    cells.header = og.header
    cells.cell_width = resolution
    cells.cell_height = resolution

    for i in range(0, height):  # height should be set to hieght of grid
        for j in range(0, width):  # width should be set to width of grid
            # print k # used for debugging
            if og.data[k] < 30:
                cells.cells.append(getPoint((j, i), resolution, offsetX, offsetY))
            k += 1
    return cells

def global_map_to_navigable_grid_cell(og):
    resolution = og.info.resolution
    width = og.info.width
    height = og.info.height
    offsetX = og.info.origin.position.x
    offsetY = og.info.origin.position.y
    radius = rospy.get_param('/move_base/global_costmap/robot_radius')

    # resolution and offset of the map
    k = 0

    all_cell = []
    bad_cell = []

    for i in range(0, height):  # height should be set to hieght of grid
        for j in range(0, width):  # width should be set to width of grid
            all_cell.append((j, i))
            if og.data[k] == -1 or og.data[k] > 50:
                bad_cell.append((j, i))
            k += 1

    for bad_cell_elem in bad_cell:
        offset = [(0, 0), (1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1)]
        offset = [tuple([radius * (1./resolution) * x for x in t]) for t in offset]
        for offset_elem in offset:
            expanded_bad_cell_elem = [sum(x) for x in zip(bad_cell_elem, offset_elem)]
            expanded_bad_cell_elem = [int(math.copysign(x,math.ceil(abs(x)))) for x in expanded_bad_cell_elem]
            expanded_bad_cell_elem = [int(x) for x in expanded_bad_cell_elem]
            t = tuple(expanded_bad_cell_elem)
            try:
                all_cell.remove(t)
            except Exception as detail:
                pass

    cells = GridCells()
    cells.header = og.header
    cells.cell_width = resolution
    cells.cell_height = resolution

    for elem in all_cell:
        cells.cells.append(getPoint(elem, resolution, offsetX, offsetY))

    return cells


def global_map_callback(og, publisher):
    gc = global_map_to_navigable_grid_cell(og)
    publisher.publish(gc)

# reads in global map
def mapCallBack(og, publisher):
    resolution = og.info.resolution
    width = og.info.width
    height = og.info.height
    offsetX = og.info.origin.position.x
    offsetY = og.info.origin.position.y

    # resolution and offset of the map
    k = 0
    cells = GridCells()
    cells.header = og.header
    cells.cell_width = resolution
    cells.cell_height = resolution

    for i in range(0, height):  # height should be set to hieght of grid
        for j in range(0, width):  # width should be set to width of grid
            # print k # used for debugging
            if og.data[k] < 30:
                cells.cells.append(getPoint((j, i), resolution, offsetX, offsetY))
            k += 1
    publisher.publish(cells)


# convert from grid position (x,y) to real world coordinate. return a Point.
def getPoint(gridpos, resolution, offsetX, offsetY):
    point = Point()
    # added secondary offset
    point.x = (gridpos[0] * resolution) + offsetX + (.5 * resolution)
    # added secondary offset ... Magic ?
    point.y = (gridpos[1] * resolution) + offsetY + (.5 * resolution)
    point.z = 0

    return point


def run():
    global pubGlobal
    global pubLocal

    pubLocal = rospy.Publisher("/navigable_points_local", GridCells, queue_size=1)
    pubGlobal = rospy.Publisher("/navigable_points_global", GridCells, queue_size=1, latch=True)

    rospy.init_node('ObstacleExpander')
    subLocal = rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, mapCallBack, callback_args=pubLocal)

    # I give up. Global cost map update thing is PITA.
    subGlobal = rospy.Subscriber("/map", OccupancyGrid, global_map_callback,
                                 callback_args=pubGlobal)

    rospy.sleep(1)

    print 'Obstacle Expander running...'

    while (1 and not rospy.is_shutdown()):
        rospy.spin()


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
