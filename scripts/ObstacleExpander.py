#!/usr/bin/env python

import rospy
from nav_msgs.msg import GridCells
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from map_msgs.msg import OccupancyGridUpdate
import tf
import numpy
import math
from CostmapThing import CostmapThing


def run():
    globalCostmapThing = CostmapThing()
    localCostmapThing = CostmapThing()

    pub_local = rospy.Publisher("/navigable_points_local", GridCells, queue_size=1)
    pub_global = rospy.Publisher("/navigable_points_global", GridCells, queue_size=1)

    rospy.init_node('ObstacleExpander')
    sub_local = rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, localCostmapThing.map_cb)
    sub_local_update = rospy.Subscriber("/move_base/local_costmap/costmap_updates", OccupancyGridUpdate,
                                        localCostmapThing.map_update_cb, callback_args=pub_local)

    sub_global = rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, globalCostmapThing.map_cb)
    sub_global_update = rospy.Subscriber("/move_base/global_costmap/costmap_updates", OccupancyGridUpdate,
                                         globalCostmapThing.map_update_cb, callback_args=pub_global)

    rospy.sleep(1)

    print 'Obstacle Expander running...'

    while (1 and not rospy.is_shutdown()):
        rospy.spin()


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
