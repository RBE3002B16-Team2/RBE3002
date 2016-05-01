import copy
import numpy

import rospy
from nav_msgs.msg import GridCells
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path
import tf
import math
from FuckTheTime import fuck_the_time
from astar import getNeighbors, getNeighborsByRadius
from Frontier import Frontier


class FrontierExplorer:
    def __init__(self, tf_listener=None, threshold=50):
        self.onedmap = None
        self.tf_listener = tf_listener
        self.threshold = threshold
        self.frontier_pub = rospy.Publisher("/frontiers", GridCells, queue_size=1)
        self.frontier_center_pub = rospy.Publisher("/frontiers_center", GridCells, queue_size=1)
        self.bad_pub = rospy.Publisher("/bad", GridCells, queue_size=1)
        self.actual_nav_pub = rospy.Publisher("/actual_nav", GridCells, queue_size=1)
        self.nav_goal_candidates_pub = rospy.Publisher("/nav_goal_candidates", GridCells, queue_size=1)

        self.odom = None
        self.og = None

        rospy.Subscriber("/map", OccupancyGrid, self.map_cb)
        rospy.Subscriber("/odom", Odometry, self.odom_cb)

        while self.odom is None or self.og is None:
            rospy.sleep(0.1)

        print "odom and map received"

    def map_cb(self, og):
        self.og = og
        self.onedmap = list(og.data)

    def odom_cb(self, odom):
        self.odom = odom

    def get_nav_goal_point_candidates(self):
        nav, unknown, obstacles = self.make_gridpos_lists()
        set_of_frontier_gridpos = self.get_set_of_frontier_gridpos(nav, unknown)
        list_of_frontiers = self.get_list_of_frontiers(set_of_frontier_gridpos)

        list_of_big_enough_froniers = filter_small_frontiers(list_of_frontiers, 8)

        bad_cells = set()

        for f in set_of_frontier_gridpos | obstacles:
            bad_cells.update(getNeighborsByRadius(f, 4, including_me=True))

        centers = [f.get_center() for f in list_of_big_enough_froniers]
        active_grid_pos = []
        for f in list_of_big_enough_froniers:
            active_grid_pos += list(f.gridpos_set)
        self.publishPoints(self.frontier_center_pub, centers)
        self.publishPoints(self.frontier_pub, active_grid_pos)
        self.publishPoints(self.bad_pub, bad_cells)

        nav_actual = nav - bad_cells
        self.publishPoints(self.actual_nav_pub, nav_actual)

        my_gridpos = self.get_my_gridpos()

        # goal_gridpos_canidates = [find_closest_gridpos(c, nav_actual) for c in centers if ]

        goal_gridpos_candidates_with_frontier = []
        for i, c in enumerate(centers):
            gp, d = find_closest_gridpos(c, nav_actual)
            if d < 10:
                goal_gridpos_candidates_with_frontier.append((gp, list_of_big_enough_froniers[i]))

        goal_gridpos_candidates_with_frontier.sort(
            key=lambda x: self.cost_goal(my_gridpos, x), reverse=False)
        goal_gridpos_candidates = [x[0] for x in goal_gridpos_candidates_with_frontier]
        goal_point_candidates = self.publishPoints(self.nav_goal_candidates_pub, goal_gridpos_candidates)

        print "Found {} frontiers, {} big enough frontiers".format(len(list_of_frontiers),
                                                                   len(list_of_big_enough_froniers))
        return goal_point_candidates

    def cost_goal(self, my_gridpos, x):
        dist = get_distance_gridpos(x[0], my_gridpos)
        low_dist_pentalty_th = 10
        if dist < low_dist_pentalty_th:
            dist = abs(low_dist_pentalty_th-dist)*100
        else:
            dist -= low_dist_pentalty_th

        return -len(x[1]) * 20 + dist
        # return get_distance_gridpos(x[0], my_gridpos)

    def make_gridpos_lists(self):
        width = self.og.info.width
        height = self.og.info.height

        k = 0
        nav_cells = set()
        unknown_cells = set()
        obstacles = set()

        for i in range(0, height):  # height should be set to hieght of grid
            for j in range(0, width):  # width should be set to width of grid
                if self.onedmap[k] == -1:
                    unknown_cells.add((j, i))
                elif self.onedmap[k] < self.threshold:
                    nav_cells.add((j, i))
                else:
                    obstacles.add((j, i))
                k += 1

        return nav_cells, unknown_cells, obstacles

    def publishPoints(self, pub, listofgridpos):
        resolution = self.og.info.resolution
        width = self.og.info.width
        height = self.og.info.height
        offsetX = self.og.info.origin.position.x
        offsetY = self.og.info.origin.position.y

        cells = GridCells()
        cells.header.frame_id = self.og.header.frame_id
        cells.cell_width = resolution
        cells.cell_height = resolution

        points = []

        for g in listofgridpos:
            point = getPoint(g, resolution, offsetX, offsetY)
            cells.cells.append(point)
            points.append(point)

        pub.publish(cells)
        return points

    def limit_max_dist(self, fr_posestamped, to_posestamped, max_dist):
        assert fr_posestamped.header.frame_id == to_posestamped.header.frame_id
        assert max_dist > 0
        angle = get_direction(fr_posestamped, to_posestamped)
        dist = get_distance(fr_posestamped, to_posestamped)
        if dist > max_dist:
            print 'next waypoint at dist ' + str(dist) + ' limiting the dist to ' + str(max_dist)
            ps = PoseStamped()
            ps.pose.position.x = (fr_posestamped.pose.position.x + math.cos(angle) * max_dist)
            ps.pose.position.y = (fr_posestamped.pose.position.y + math.sin(angle) * max_dist)
            # ps.pose.position.x = 1
            # ps.pose.position.y = 1
            ps.header.frame_id = fr_posestamped.header.frame_id

            quaternion = tf.transformations.quaternion_from_euler(0, 0, angle)
            ps.pose.orientation.x = quaternion[0]
            ps.pose.orientation.y = quaternion[1]
            ps.pose.orientation.z = quaternion[2]
            ps.pose.orientation.w = quaternion[3]

            ps = fuck_the_time(ps)
            print 'actual dist ' + str(get_distance(fr_posestamped, ps))
            return ps
        else:
            return to_posestamped

    def get_set_of_frontier_gridpos(self, nav, unknown):
        f = set()
        for nav_el in nav:
            neighbors = getNeighbors(nav_el)
            for n in neighbors:
                if n in unknown:
                    f.add(nav_el)
                    break
        return f

    def get_list_of_frontiers(self, set_of_frontier_gridpos):
        list_of_frontiers = []
        sofg = copy.deepcopy(set_of_frontier_gridpos)
        while sofg:
            start = sofg.pop()
            gridpos_set = self.fill_frontier_mkii(start, sofg)
            frontier = Frontier(gridpos_set=gridpos_set)
            list_of_frontiers.append(frontier)
        return list_of_frontiers

    def fill_frontier_mkii(self, start, set_of_frontier_gridpos, cluster=None):
        if cluster is None:
            cluster = set()
        set_of_frontier_gridpos.discard(start)
        cluster.add(start)
        neighbors = getNeighbors(start, set_of_frontier_gridpos)
        for n in neighbors:
            self.fill_frontier_mkii(n, set_of_frontier_gridpos, cluster)
        return cluster

    def get_my_gridpos(self):
        current_pose_stamped_in_map = self.get_my_posestamped()
        my_gridpos = pose2gridpos_og(current_pose_stamped_in_map.pose, self.og)
        return my_gridpos

    def get_my_posestamped(self):
        self.tf_listener.waitForTransform('map', 'odom', rospy.Time(0), rospy.Duration(10.0))
        ps = PoseStamped()
        ps.pose = self.odom.pose.pose
        ps.header = self.odom.header
        current_pose_stamped_in_map = self.tf_listener.transformPose('/map', fuck_the_time(ps))
        return current_pose_stamped_in_map


def find_closest_gridpos(me, in_where):
    nodes = numpy.asarray(list(in_where))
    me = numpy.asarray(me)
    dist_squared = numpy.sum((nodes - me) ** 2, axis=1)
    i = numpy.argmin(dist_squared)
    return tuple(nodes[i]), numpy.sqrt(dist_squared[i])


def filter_small_frontiers(frontiers, min_size):
    return [f for f in frontiers if f.get_size() >= min_size]


def get_direction_gridpos(fr, to):
    dx = to[0] - fr[0]
    dy = to[1] - fr[1]
    return math.atan2(dy, dx)


def get_distance_gridpos(fr, to):
    dx = to[0] - fr[0]
    dy = to[1] - fr[1]
    return math.sqrt(dx ** 2 + dy ** 2)


def get_distance(fr_posestamped, to_posestamped):
    dx = fr_posestamped.pose.position.x - to_posestamped.pose.position.x
    dy = fr_posestamped.pose.position.y - to_posestamped.pose.position.y
    return math.sqrt(math.pow(dx, 2) + math.pow(dy, 2))


def get_direction(fr_posestamped, to_posestamped):
    dx = to_posestamped.pose.position.x - fr_posestamped.pose.position.x
    dy = to_posestamped.pose.position.y - fr_posestamped.pose.position.y
    return math.atan2(dy, dx)


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


def pose2gridpos_og(pose, og):
    resolution = og.info.resolution
    width = og.info.width
    height = og.info.height
    offsetX = og.info.origin.position.x
    offsetY = og.info.origin.position.y
    return pose2gridpos(pose, resolution, offsetX, offsetY)
