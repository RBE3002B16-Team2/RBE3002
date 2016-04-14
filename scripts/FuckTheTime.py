import rospy


def fuck_the_time(pose_stamped):
    pose_stamped.header.stamp = rospy.Time(0)
    return pose_stamped