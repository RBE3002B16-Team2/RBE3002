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

class Final3002:
    def __init__(self):
        self.tfl = tf.TransformListener()
        self.fe = FrontierExplorer(tf_listener=self.tfl)

        self.nav_goal_candidates_pub = rospy.Publisher("/next_goal", PoseStamped, queue_size=1)

    def test_get_goal_points(self,event):
        print "test get goal points"
        points = self.fe.get_nav_goal_point_candidates()

        next_goal = PoseStamped()
        next_goal.pose.position = points.pop(0)
        next_goal.header.frame_id = 'map'
        next_goal = fuck_the_time(next_goal)

        self.nav_goal_candidates_pub.publish(next_goal)



if __name__ == '__main__':
    print "Starting RBE3002 final"
    rospy.init_node('rbe3002_final')
    inst = Final3002()

    rospy.Timer(rospy.Duration(5), inst.test_get_goal_points)

    while not rospy.is_shutdown():
        rospy.spin()

