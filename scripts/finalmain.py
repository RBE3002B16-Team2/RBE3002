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



from FrontierExplorer import FrontierExplorer
from MoveBaseThingy import MoveBaseThingy
from actionlib_msgs.msg import GoalStatus

class Final3002:
    def __init__(self):
        self.tfl = tf.TransformListener()
        self.fe = FrontierExplorer(tf_listener=self.tfl)
        self.move = MoveBaseThingy()

        self.next_goal_pub = rospy.Publisher("/next_goal", PoseStamped, queue_size=1)

    def explore(self):
        print 'start explore'
        points = self.fe.get_nav_goal_point_candidates()

        while points:
            r = self.go_to_first_feasible_point(points)
            if r is False:
                print "Got stuck. give up"
                return
            points = self.fe.get_nav_goal_point_candidates()

        print 'done explore'

    def go_to_first_feasible_point(self,points):
        while points:
            p = points.pop(0)
            next_goal = point2pose(p)
            self.next_goal_pub.publish(next_goal)
            result = self.move.go_to(next_goal)
            if result in [GoalStatus.SUCCEEDED, GoalStatus.ABORTED]:
                return True
            elif result in [GoalStatus.REJECTED]:
                continue
            else:
                print "This should not happen. move_base result " + repr(result)

        print "None of the points are feasible"
        return False


    def test_get_goal_points(self,event):
        print "test get goal points"
        points = self.fe.get_nav_goal_point_candidates()

        next_goal = PoseStamped()
        next_goal.pose.position = points.pop(0)
        next_goal.header.frame_id = 'map'
        next_goal = fuck_the_time(next_goal)

        self.next_goal_pub.publish(next_goal)


def point2pose(point):
    next_goal = PoseStamped()
    next_goal.pose.position = point
    q = tf.transformations.quaternion_from_euler(0, 0, 0)
    next_goal.pose.orientation.x = q[0]
    next_goal.pose.orientation.y = q[1]
    next_goal.pose.orientation.z = q[2]
    next_goal.pose.orientation.w = q[3]
    next_goal.header.frame_id = 'map'
    next_goal = fuck_the_time(next_goal)
    return next_goal

if __name__ == '__main__':
    print "Starting RBE3002 final"
    rospy.init_node('rbe3002_final')
    inst = Final3002()

    # rospy.Timer(rospy.Duration(5), inst.test_get_goal_points)

    # rospy.sleep(1)
    inst.explore()

    while not rospy.is_shutdown():
        rospy.spin()

