import rospy
import actionlib
import tf
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import Pose, Point
from actionlib_msgs.msg import GoalStatus
import math

class MoveBaseThingy:
    def __init__(self):
        self.movebase = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        print 'waiting for move base...'
        self.movebase.wait_for_server()
        print 'move base action server found'

    '''
    PoseStamped goal
    returns int GoalStatus from the status
    This method should block until nav done or aborted
    '''

    def go_to(self, goal_posestamped, timeout=0):
        goal = MoveBaseGoal()
        goal.target_pose = goal_posestamped
        self.movebase.send_goal(goal)
        self.movebase.wait_for_result()  # timeout maybe?
        result = self.movebase.get_goal_status_text()
        print 'move_base ' + result
        resultcode = self.movebase.get_state()
        print 'result code ' + repr(resultcode)
        return resultcode

    '''
    spin about 360 degrees
    blocks until done
    '''

    def spin(self, currentpose):
        for angle in [0, math.pi*2/3, math.pi*4/3, math.pi*2]:
            quat = tf.transformations.quaternion_from_euler(0, 0, angle)
            currentpose.pose.orientation.x = quat[0]
            currentpose.pose.orientation.y = quat[1]
            currentpose.pose.orientation.z = quat[2]
            currentpose.pose.orientation.w = quat[3]
            self.go_to(currentpose)
