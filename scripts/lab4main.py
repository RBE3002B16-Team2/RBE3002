#!/usr/bin/env python

# Author Joseph St. Germain
# Co-Authur Arthur lockmans drive smooth function


import rospy, tf, numpy, math
from kobuki_msgs.msg import BumperEvent
from map_msgs.msg import OccupancyGridUpdate
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry, Path, GridCells, OccupancyGrid
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

from CostmapThing import CostmapThing
from astar import NoPathFoundException
from FuckTheTime import fuck_the_time

wheel_rad = 3.5 / 100.0  # cm
wheel_base = 23.0 / 100.0  # cm


def publishTwist(lin_Vel, ang_Vel):
    """Send a movement (twist) message."""
    global pub
    msg = Twist()
    msg.linear.x = lin_Vel
    msg.angular.z = ang_Vel
    pub.publish(msg)


def navToPose(whatever_frame_goal):
    """Drive to a goal subscribed to from /move_base_simple/goal"""
    # compute angle required to make straight-line move to desired pose
    global xPosition
    global yPosition
    global theta
    global odom_list

    odom_list.waitForTransform('odom', 'map', rospy.Time(0), rospy.Duration(10.0))
    goal = odom_list.transformPose('map', fuck_the_time(whatever_frame_goal))

    initialX = xPosition
    initialY = yPosition
    initialT = theta

    # capture desired x and y positions
    desiredY = goal.pose.position.y
    desiredX = goal.pose.position.x
    # capture desired angle
    quat = goal.pose.orientation
    q = [quat.x, quat.y, quat.z, quat.w]
    roll, pitch, yaw = euler_from_quaternion(q)
    desiredT = yaw

    dx = desiredX - initialX
    dy = desiredY - initialY
    distance = math.sqrt((dx) ** 2 + (dy) ** 2)
    pathT = math.atan2(dy, dx)

    initialTurn = getMinimalAngleDifference(pathT, initialT)
    finalTurn = getMinimalAngleDifference(desiredT, pathT)

    rotate(initialTurn)


    driveSmooth(0.25, distance)
    rospy.sleep(0.5)
    rotate(finalTurn)


def navToPose_but_dont_care_end_orientation(whatever_frame_goal):
    """Drive to a goal subscribed to from /move_base_simple/goal"""
    # compute angle required to make straight-line move to desired pose
    global xPosition
    global yPosition
    global theta
    global odom_list

    odom_list.waitForTransform('odom', 'map', rospy.Time(0), rospy.Duration(10.0))
    goal = odom_list.transformPose('map', fuck_the_time(whatever_frame_goal))

    initialX = xPosition
    initialY = yPosition
    initialT = theta

    # capture desired x and y positions
    desiredY = goal.pose.position.y
    desiredX = goal.pose.position.x
    # capture desired angle
    quat = goal.pose.orientation
    q = [quat.x, quat.y, quat.z, quat.w]
    roll, pitch, yaw = euler_from_quaternion(q)
    desiredT = yaw

    dx = desiredX - initialX
    dy = desiredY - initialY
    distance = math.sqrt((dx) ** 2 + (dy) ** 2)
    pathT = math.atan2(dy, dx)

    initialTurn = getMinimalAngleDifference(pathT, initialT)
    finalTurn = getMinimalAngleDifference(desiredT, pathT)

    rotate(initialTurn)
    driveSmooth(0.25, distance)


def getMinimalAngleDifference(target, source):
    return math.atan2(math.sin(target - source), math.cos(target - source))


def driveSmooth(speed, distance):
    """This function accepts a speed and a distance for the robot to move in a smoothed straight line."""
    global pose

    initialX = pose.position.x
    initialY = pose.position.y
    atTarget = False
    currentSpeed = 0.
    currentDistance = 0.
    sampleTime = 0.15
    acceleration = 0.5  # unit/s
    # Loop until the distance between the attached frame and the origin is equal to the
    # distance specified
    while (not atTarget and not rospy.is_shutdown()):
        if (currentSpeed < speed):
            currentSpeed = currentSpeed + acceleration / (1. / sampleTime)
        else:
            currentSpeed = speed

        maxSpeedDeceleration = (distance - currentDistance) * 0.5 + 0.1
        if (currentSpeed > maxSpeedDeceleration):
            currentSpeed = maxSpeedDeceleration

        currentX = pose.position.x
        currentY = pose.position.y
        currentDistance = math.sqrt((currentX - initialX) ** 2 + (currentY - initialY) ** 2)  # Distance formula
        if (currentDistance >= distance):
            atTarget = True
            publishTwist(0, 0)
        else:
            publishTwist(currentSpeed, 0)
            rospy.sleep(sampleTime)


def rotate(angle):
    global odom_list
    global pose
    vel = Twist();
    done = True
    targetAngle = pose.orientation.z + angle
    error = getMinimalAngleDifference(targetAngle, pose.orientation.z)
    while ((abs(error) >= 0.05) and not rospy.is_shutdown()):
        publishTwist(0, numpy.sign(error) * math.pi * 0.2)
        rospy.sleep(0.15)
        error = getMinimalAngleDifference(targetAngle, pose.orientation.z)
    publishTwist(0, 0)


# keeps track of current location and orientation
def tCallback(event):
    global pose
    global xPosition
    global yPosition
    global theta

    odom_list.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(10.0))
    (position, orientation) = odom_list.lookupTransform('map', 'base_footprint', rospy.Time(0))
    pose.position.x = position[0]
    pose.position.y = position[1]
    # the previous 2 lines and next 2 lines are repedative. Joes bad
    xPosition = position[0]
    yPosition = position[1]

    odomW = orientation
    q = [odomW[0], odomW[1], odomW[2], odomW[3]]
    roll, pitch, yaw = euler_from_quaternion(q)
    # convert yaw to degrees
    pose.orientation.z = yaw
    theta = yaw


def pathToPoints(list_of_pos):
    print "Path to points..."
    for pose in list_of_pos.poses:
        navToPose(pose)


def plan_a_path_and_nav_to_goal(goal):
    global pose
    global globalCostmapThing
    global localCostmapThing
    global odom_list
    global pubintermediategoalpose
    global where_the_f_am_i_going
    global pubglobalwp
    global pubglobalpath

    ps = PoseStamped()
    ps.pose = pose
    ps.header.frame_id = 'map'
    ps.header.stamp = rospy.Time(0)

    localdone = False
    globaldone = False

    nextwp = None

    local_feasible = True
    global_feasible = True

    while not globaldone:
        try:
            print 'planning for global map'
            nextwp, globaldone = globalCostmapThing.getNextWaypoint(ps, goal, odom_list, wppub=pubglobalwp, pathpub=pubglobalpath, skip=1)
            global_feasible = True
            localdone = False
            while not localdone:
                try:
                    print 'tryin to replan in the local map'
                    # nextwp_l, localdone = localCostmapThing.getNextWaypoint(ps, nextwp, odom_list, pathpub=pubrealpath,
                    #                                                         dist_limit=1,
                    #                                                         wppub=pubintermediategoalpose, skip=2)
                    # nextwp = localCostmapThing.getNextWaypoint(ps, goal, odom_list, pathpub=pubrealpath,
                    #                                            wppub=pubintermediategoalpose, dist_limit=0.5)

                    if global_feasible and not local_feasible:
                        print 'found path in global but no path in local. go to global path.'
                        navToPose_but_dont_care_end_orientation(nextwp)

                    list_of_pose = localCostmapThing.get_a_list_of_pose_to_goal(ps, nextwp, odom_list, pathpub=pubrealpath,
                                                      dist_limit=1.0,
                                                      wppub=pubintermediategoalpose)
                    local_feasible = True
                    for i, nextwp_l in enumerate(list_of_pose):
                        if 0 < i < 2:
                            odom_list.waitForTransform('odom', 'map', rospy.Time(0), rospy.Duration(10.0))
                            nextwp_lt = odom_list.transformPose('map', fuck_the_time(nextwp_l))
                            where_the_f_am_i_going.publish(nextwp_lt)
                            print 'naving the local waypoints'
                            navToPose_but_dont_care_end_orientation(nextwp_lt)
                            localdone = True
                    # nextwp = nextwp_l
                except NoPathFoundException:
                    local_feasible = False
                    print 'no path found in local. replan in global'
                    try:
                        nextwp, globaldone = globalCostmapThing.getNextWaypoint(ps, goal, odom_list, wppub=pubglobalwp, pathpub=pubglobalpath, skip=1)
                        global_feasible = True
                    except:
                        global_feasible = False
                        print 'no path found to goal. give up'
                        return
            print 'Reached local goal'

        except NoPathFoundException:
            global_feasible = False
            print 'cannot find a path in global map'
            return
    navToPose(goal)
    print 'Reached global goal'


# This is the program's main function
if __name__ == '__main__':
    rospy.init_node('lab4')
    global pub
    global odom_list

    global pose
    global globalCostmapThing
    global localCostmapThing
    global pubrealpath
    global pubintermediategoalpose
    global where_the_f_am_i_going
    global pubglobalwp
    global pubglobalpath

    # global odom_tf
    odom_list = tf.TransformListener()  # listner for robot location

    pose = Pose()

    pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, None,
                          queue_size=10)  # Publisher for commanding robot motion

    goal_sub = rospy.Subscriber('/move_base_simple/goalrbe', PoseStamped, plan_a_path_and_nav_to_goal, queue_size=1)

    pubpath = rospy.Publisher("/path", GridCells, queue_size=1)
    pubopen = rospy.Publisher("/opennodes", GridCells, queue_size=1)
    pubclose = rospy.Publisher("/closednodes", GridCells, queue_size=1)

    globalCostmapThing = CostmapThing(odom_list, astarpubs=(pubopen, pubclose, pubpath), threshold=40)
    # globalCostmapThing = CostmapThing(odom_list)

    localCostmapThing = CostmapThing(odom_list, astarpubs=(pubopen, pubclose, pubpath), threshold=50)

    pub_local = rospy.Publisher("/navigable_points_local", GridCells, queue_size=1)
    pub_global = rospy.Publisher("/navigable_points_global", GridCells, queue_size=1)

    sub_local = rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, localCostmapThing.map_cb)
    sub_local_update = rospy.Subscriber("/move_base/local_costmap/costmap_updates", OccupancyGridUpdate,
                                        localCostmapThing.map_update_cb, callback_args=pub_local)

    sub_global = rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, globalCostmapThing.map_cb)
    sub_global_update = rospy.Subscriber("/move_base/global_costmap/costmap_updates", OccupancyGridUpdate,
                                         globalCostmapThing.map_update_cb, callback_args=pub_global)

    pubrealpath = rospy.Publisher("/realpath", Path, queue_size=1)

    pubintermediategoalpose = rospy.Publisher("/pubintermediategoalpose", PoseStamped, queue_size=1)
    pubglobalwp = rospy.Publisher("/globalwp", PoseStamped, queue_size=1)
    pubglobalpath = rospy.Publisher("/globalpath", Path, queue_size=1)

    where_the_f_am_i_going = rospy.Publisher("/where_the_f_am_i_going", PoseStamped, queue_size=1)
    rospy.Timer(rospy.Duration(.01), tCallback)  # timer callback for robot location

    print "Starting Lab 4"
    while not rospy.is_shutdown():
        rospy.spin()
