import copy
import rospy
from nav_msgs.msg import GridCells
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path
import tf
import math
from FuckTheTime import fuck_the_time
from astar import getNeighbors
from Frontier import Frontier
import sys


class FrontierExplorer:
    def __init__(self, tf_listener=None, threshold=50):
        self.onedmap = None
        self.tf_listener = tf_listener
        self.threshold = threshold
        self.frontier_pub = rospy.Publisher("/frontiers", GridCells, queue_size=1)
        self.frontier_center_pub = rospy.Publisher("/frontiers_center", GridCells, queue_size=1)
        rospy.Subscriber("/map", OccupancyGrid, self.map_cb)

    def map_cb(self, og):
        self.og = og
        self.onedmap = list(og.data)
        self.publish_frontiers()

    def publish_frontiers(self):
        nav,unknown = self.make_gridpos_lists()
        set_of_frontier_gridpos = self.get_set_of_frontier_gridpos(nav,unknown)
        list_of_frontiers = self.get_list_of_frontiers(set_of_frontier_gridpos)

        centers = [f.get_center() for f in list_of_frontiers]
        for f in list_of_frontiers:
            pass
            # print f.gridpos_set
        # print centers
        active_grid_pos = []
        for f in list_of_frontiers:
            active_grid_pos += list(f.gridpos_set)
        self.publishPoints(self.frontier_center_pub, centers)
        self.publishPoints(self.frontier_pub, active_grid_pos)

        # assert len(active_grid_pos) == len(set(active_grid_pos))

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
                if self.onedmap[k] < self.threshold:
                    cells.cells.append(getPoint((j, i), resolution, offsetX, offsetY))
                k += 1

        return cells

    def make_gridpos_lists(self):
        width = self.og.info.width
        height = self.og.info.height

        k = 0
        nav_cells = set()
        unknown_cells = set()

        for i in range(0, height):  # height should be set to hieght of grid
            for j in range(0, width):  # width should be set to width of grid
                if self.onedmap[k] == -1:
                    unknown_cells.add((j, i))
                elif self.onedmap[k] < self.threshold:
                    nav_cells.add((j, i))
                k += 1

        return nav_cells, unknown_cells

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

        for g in listofgridpos:
            point = getPoint(g, resolution, offsetX, offsetY)
            cells.cells.append(point)

        pub.publish(cells)

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



#Functions thats need to be found/written/recycled/implemented:
#    get_distanceToFrontier(farthestFrontier) takes in a frontier and returns the distance to that frontoier from the robots current location
#    get_poseOfFrontier(farthestFrontier, divisor)) takes in a frontier and returns a pose in which the x and y are divided by the divisor
#    convertToStampedPose(pose) takes in a pose and returns a stamped pose
#    planToFrontierExists(frontierInQuestion) returns true of a navigatalbe path exsists to a given stampedPose
#    move_base(stapedPose) moves the robot to the given stampedPose

def nav2Frontier(frontiers):
    #Sets the given list of frontiers to a a temp list
    tempFrontier = frontiers
    pathHasntBeenFound = true
    
    #Repeats until a navigatable path has been found 
    while(pathHasntBeenFound and (len(tempFrontier)>1)):
        
        #Sets the number of path planing attempts for the frontier in question equal to zero
        pathPlainingAttempts = 0
       
        #Gets the fartherst frontier in the list
        for f in tempFrontier:
            if(get_distanceToFrontier(f, currentRobotPoint) > get_distanceToFrontier(farthestFrontier, currentRobotPoint)):
                farthestFrontier = f
                
        #
        while(pathPlainingAttempts < 4):
            frontierInQuestion  = convertToStampedPose(get_poseOfFrontier(farthestFrontier, pathPlainingAttempts+2))
            
        
            if(planToFrontierExists(frontierInQuestion)):
                pathHasntBeenFound = false
            #    move_base to frontierInQuestion
            else:
                pathPlainingAttempts+=1
                frontierInQuestion  = convertToStampedPose(get_poseOfFrontier(farthestFrontier, pathPlainingAttempts+2))
                
        
        tempFrontier.remove(frontierInQuestion)
    
    print 'Error'
   
    



def getDirection(fr, to):
    dx = to[0] - fr[0]
    dy = to[1] - fr[1]
    return math.atan2(dy, dx)


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
