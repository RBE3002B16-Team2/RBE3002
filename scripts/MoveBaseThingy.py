import actionlib
import rospy

class MoveBaseThingy:
    def __init__(self):
        pass

    '''
    PoseStamped goal
    returns int GoalStatus from the status
    This method should block until nav done or aborted
    '''
    def go_to(self, nav_pose, timeout=0):
        move_base_client = actionlib.SimpleActionClient('move_base_local', MoveBaseAction)
        move_base_client.wait_for_server()
        goal = MoveBaseGoal()
        goal.target_pose = nav_pose
        #send the goal and wait for the base to get there
        move_base_client.send_goal_and_wait(goal)


    '''
    spin about 360 degrees
    blocks until done
    '''
    def spin(self):
        pass

if __name__ == '__main__':
    print "Starting MoveBaseThingy"
    rospy.init_node('MoveBaseThingy')
    rospy.sleep(10)
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'base_link'
    goal.target_pose.pose.position.x = 1.0
    goal.target_pose.pose.orientation.w = 1.0
    go_to(goal, 10)
    while not rospy.is_shutdown():
        rospy.spin()
