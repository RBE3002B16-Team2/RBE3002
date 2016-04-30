import rospy
import actionlib
import tf
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import Pose, Point
from actionlib_msgs.msg import GoalStatus


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
        return resultcode


    '''
    spin about 360 degrees
    blocks until done
    '''
    def spin(self):
        pass