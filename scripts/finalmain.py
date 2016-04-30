import rospy, tf, numpy, math

from FrontierExplorer import FrontierExplorer

if __name__ == '__main__':
    print "Starting RBE3002 final"
    rospy.init_node('rbe3002_final')
    fe = FrontierExplorer()
    rospy.sleep()
    while not rospy.is_shutdown():
        rospy.spin()

