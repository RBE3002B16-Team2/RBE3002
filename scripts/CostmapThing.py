import copy
import rospy
from nav_msgs.msg import GridCells
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path
import tf
import math
from astar import aStar, NoPathFoundException


class CostmapThing:
    def __init__(self):
        self.onedmap = None

    def map_cb(self, og):
        self.og = og
        self.onedmap = list(og.data)

    def map_update_cb(self, ogu, publisher=None):
        if self.onedmap is None:
            raise Exception("global map update aborted because no initial map received")
        map_index = 0
        update_index = 0
        from_x = ogu.x
        from_y = ogu.y
        to_x = from_x + ogu.width - 1
        to_y = from_y + ogu.height - 1
        for row in range(0, self.og.info.height):
            for col in range(0, self.og.info.width):
                if from_x <= col <= to_x and from_y <= row <= to_y:
                    self.onedmap[map_index] = ogu.data[update_index]
                    update_index += 1
                map_index += 1
        if publisher is not None:
            publisher.publish(self.make_grid_cell())

    def make_grid_cell(self):
        resolution = self.og.info.resolution
        width = self.og.info.width
        height = self.og.info.height
        offsetX = self.og.info.origin.position.x
        offsetY = self.og.info.origin.position.y

        # resolution and offset of the map
        k = 0
        cells = GridCells()
        cells.header = self.og.header
        cells.cell_width = resolution
        cells.cell_height = resolution

        for i in range(0, height):  # height should be set to hieght of grid
            for j in range(0, width):  # width should be set to width of grid
                if self.onedmap[k] < 30:
                    cells.cells.append(getPoint((j, i), resolution, offsetX, offsetY))
                k += 1

        return cells

    def make_navigable_gridpos(self):
        width = self.og.info.width
        height = self.og.info.height

        # resolution and offset of the map
        k = 0
        cells = []

        for i in range(0, height):  # height should be set to hieght of grid
            for j in range(0, width):  # width should be set to width of grid
                if self.onedmap[k] < 30:
                    cells.append((j, i))
                k += 1

        return cells

    # returns PoseStamped
    def getNextWaypoint(self, start, goal, tf_listener, dist_limit=None, pathpub=None, wppub=None):
        resolution = self.og.info.resolution
        width = self.og.info.width
        height = self.og.info.height
        offsetX = self.og.info.origin.position.x
        offsetY = self.og.info.origin.position.y

        tf_listener.waitForTransform('map', 'odom', rospy.Time(0), rospy.Duration(1.0))
        tstart = tf_listener.transformPose(self.og.header.frame_id, start)
        tgoal = tf_listener.transformPose(self.og.header.frame_id, goal)

        if dist_limit is not None:
            tgoal = limit_max_dist(tstart, tgoal, dist_limit)

        if wppub is not None:
            wppub.publish(tgoal)

        start_grid_pos = pose2gridpos(tstart.pose, resolution, offsetX, offsetY)
        goal_grid_pos = pose2gridpos(tgoal.pose, resolution, offsetX, offsetY)
        try:
            path = aStar(self.make_navigable_gridpos(), start_grid_pos, goal_grid_pos)
            wp = self.getWaypoints(path, goal, pathpub)
            retp = PoseStamped()
            if len(wp) > 1:
                retp.pose = wp[1].pose
                retp.header = self.og.header
            else:
                retp = goal
            return retp
        except NoPathFoundException as e:
            raise e

    # generate a Path from a list of gridpos (x,y)
    def getWaypoints(self, list_of_gridpos, goal_pose, publisher=None):
        resolution = self.og.info.resolution
        width = self.og.info.width
        height = self.og.info.height
        offsetX = self.og.info.origin.position.x
        offsetY = self.og.info.origin.position.y

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
                newPose.pose.position = getPoint(list_of_gridpos[i], resolution, offsetX, offsetY)
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
        if publisher is not None:
            publisher.publish(wp)
        return wp.poses


def getDirection(fr, to):
    dx = to[0] - fr[0]
    dy = to[1] - fr[1]
    return math.atan2(dy, dx)


def get_distance(fr_posestamped, to_posestamped):
    dx = fr_posestamped.pose.position.x - to_posestamped.pose.position.x
    dy = fr_posestamped.pose.position.y - to_posestamped.pose.position.y
    return math.sqrt(dx ** 2 + dy ** 2)


def get_direction(fr_posestamped, to_posestamped):
    dx = fr_posestamped.pose.position.x - to_posestamped.pose.position.x
    dy = fr_posestamped.pose.position.y - to_posestamped.pose.position.y
    return math.atan2(dy, dx)


def limit_max_dist(fr_posestamped, to_posestamped, max_dist):
    angle = get_direction(fr_posestamped, to_posestamped)
    if get_distance(fr_posestamped, to_posestamped) > max_dist:
        to_posestamped.pose.position.x = fr_posestamped.pose.position.x + math.cos(angle) * max_dist
        to_posestamped.pose.position.y = fr_posestamped.pose.position.y + math.sin(angle) * max_dist
    return to_posestamped


def getPoint(gridpos, resolution, offsetX, offsetY):
    point = Point()
    point.x = (gridpos[0] * resolution) + offsetX + (.5 * resolution)
    point.y = (gridpos[1] * resolution) + offsetY + (.5 * resolution)
    point.z = 0

    return point


def pose2gridpos(pose, resolution, offsetX, offsetY):
    gridx = int((pose.position.x - offsetX - (.5 * resolution)) / resolution)
    gridy = int((pose.position.y - offsetY - (.5 * resolution)) / resolution)
    return (gridx, gridy)
