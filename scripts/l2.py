#!/usr/bin/env python

#Author Joseph St. Germain 
#Co-Authur Arthur lockmans drive smooth function


import rospy, tf, numpy, math
from kobuki_msgs.msg import BumperEvent
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion


wheel_rad = 3.5 / 100.0 #cm
wheel_base = 23.0 / 100.0 #cm

def publishTwist(lin_Vel, ang_Vel):
    """Send a movement (twist) message."""
    global pub
    msg = Twist()
    msg.linear.x = lin_Vel
    msg.angular.z = ang_Vel
    pub.publish(msg)

def navToPose(goal):
    """Drive to a goal subscribed to from /move_base_simple/goal"""
    #compute angle required to make straight-line move to desired pose
    global xPosition
    global yPosition
    global theta
    
    initialX = xPosition
    initialY = yPosition
    initialT = theta

    #capture desired x and y positions
    desiredY = goal.pose.position.y
    desiredX = goal.pose.position.x
    #capture desired angle
    quat = goal.pose.orientation
    q = [quat.x, quat.y, quat.z, quat.w]
    roll, pitch, yaw = euler_from_quaternion(q)
    desiredT = yaw

    dx = desiredX - initialX
    dy = desiredY - initialY
    distance = math.sqrt((dx)**2 + (dy)**2)
    pathT = math.atan2(dy,dx)

    initialTurn = getMinimalAngleDifference(pathT, initialT)
    finalTurn =  getMinimalAngleDifference(desiredT, pathT)
    
    

    print "initialTurn" + str(initialTurn)
    print "finalTurn" + str(finalTurn)
    print "distance" + str(distance)
    
    print "spin!" #turn to calculated angle
    rotate(initialTurn)
    print "move!" #move in straight line specified distance to new pose
    #driveSmooth(0.25, distance)

    driveSmooth(0.25, distance)
    rospy.sleep(2)
    print "spin!" #spin to final angle 
    rotate(finalTurn)
    print "done"
    

def getMinimalAngleDifference(target, source):
    return math.atan2(math.sin(target-source),math.cos(target-source))

#This function accepts two wheel velocities and a time interval.
def spinWheels(u1, u2, time):
    """This function accepts two wheel velocities and a time interval."""
    global pub

    r = wheel_rad
    b = wheel_base
    #compute wheel speeds
    u = (r/2.)*(u1+u2)     #Determines the linear velocity of base based on the wheel
    w = (r/b)*(u1-u2)     #Determines the angular velocity of base on the wheels.
    start = rospy.Time().now().secs
    #create movement and stop messages
    move_msg = Twist() #creates a move_msg object inheriting type from the Twist() class
    move_msg.linear.x = u #sets linear velocity
    move_msg.angular.z = w #sets amgular velocity (Populates messages with data.)

    stop_msg = Twist()
    stop_msg.linear.x = 0
    stop_msg.angular.z = 0
    #publish move message for desired time
    while(rospy.Time().now().secs - start < time and not rospy.is_shutdown()): # waits for said time and checks for ctrl C
        pub.publish(move_msg) #publishes the move_msg
    pub.publish(stop_msg)

#This function accepts a speed and a distance for the robot to move in a straight line
def driveStraight(speed, distance):
    """This function accepts a speed and a distance for the robot to move in a straight line"""
    global pose

    initialX = pose.position.x
    initialY = pose.position.y
    atTarget = False

    #Loop until the distance between the attached frame and the origin is equal to the
    #distance specified 
    while (not atTarget and not rospy.is_shutdown()):
        currentX = pose.position.x
        currentY = pose.position.y
        currentDistance = math.sqrt((currentX - initialX)**2 + (currentY-initialY)**2) #Distance formula
        if (currentDistance >= distance):
            atTarget = True
            publishTwist(0, 0)
        else:
            publishTwist(speed, 0)
            rospy.sleep(0.15)


def driveSmooth(speed, distance):
    """This function accepts a speed and a distance for the robot to move in a smoothed straight line."""
    global pose

    initialX = pose.position.x
    initialY = pose.position.y
    atTarget = False
    currentSpeed = 0.
    currentDistance = 0.
    sampleTime = 0.15
    acceleration = 0.5 # unit/s
    #Loop until the distance between the attached frame and the origin is equal to the
    #distance specified 
    while (not atTarget and not rospy.is_shutdown()):
        if (currentSpeed < speed):
            currentSpeed = currentSpeed + acceleration/(1./sampleTime)
        else:
            currentSpeed = speed
        
        maxSpeedDeceleration = (distance - currentDistance)*1 + 0.1
        if (currentSpeed > maxSpeedDeceleration):
            currentSpeed = maxSpeedDeceleration

        currentX = pose.position.x
        currentY = pose.position.y
        currentDistance = math.sqrt((currentX - initialX)**2 + (currentY-initialY)**2) #Distance formula
        if (currentDistance >= distance):
            atTarget = True
            publishTwist(0, 0)
        else:
            publishTwist(currentSpeed, 0)
            rospy.sleep(sampleTime)




def rotate(angle):
    global odom_list
    global pose
    vel = Twist();   
    done = True
    targetAngle = pose.orientation.z + angle    
    error = getMinimalAngleDifference(targetAngle,pose.orientation.z)
    while ((abs(error) >= 0.05) and not rospy.is_shutdown()):
        publishTwist(0,numpy.sign(error)*math.pi*0.2)
        rospy.sleep(0.15)
        error = getMinimalAngleDifference(targetAngle,pose.orientation.z)
    publishTwist(0,0)


def executeTrajectory():
    """This function sequentially calls methods to perform a trajectory."""
    print 'executeTrajectory'
    driveSmooth(50,0.6)
    rotate(-math.pi/2)
    driveSmooth(50,0.45)
    rotate(135*math.pi/180)
    print 'end of executeTrajectory'


def driveArc(radius, speed, angle):
    """This function works the same as rotate how ever it does not publish linear velocities."""
    #assuming radius is turning radius, speed is drive speed, angle is desired final angle
    #calculate wheel speeds and time to move from current pose to final pose
    #spinWheels with time and speeds to move to correct pose
    w = speed / radius
    #v1 = w * (radius + .5*.352)
    #v2 = w * (radius - .5*.352)
    
    global odom_list
    global pose
    targetAngle = pose.orientation.z + angle    
    error = getMinimalAngleDifference(targetAngle,pose.orientation.z)
    while ((abs(error) >= 0.05) and not rospy.is_shutdown()):
        publishTwist(speed, numpy.sign(error)*w)
        rospy.sleep(0.15)
        error = getMinimalAngleDifference(targetAngle,pose.orientation.z)
        print 'z ' + str(pose.orientation.z)
        print 'error ' + str(error)
    publishTwist(0,0)

    ############################# The rest of this function will be at least as long as rotate
    pass  # Delete this 'pass' once implemented


def readBumper(msg):
    """Bumper event callback"""
    if (msg.state == 1 and msg.bumper ==1):
        # What should happen when the bumper is pressed?
        #Stop forward motion if bumper is pressed
        print "Bumper pressed!"
        executeTrajectory()
    if (msg.state == 1 and msg.bumper == 2):
        print "driveArc"
        driveArc(0.2,0.1,math.pi/2)
    if (msg.state == 1 and msg.bumper == 3):
        print "spinWheels"
        spinWheels(0.,1.,5.)


#keeps track of current location and orientation
def tCallback(event):
	
    global pose
    global xPosition
    global yPosition
    global theta

    odom_list.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(1.0))
    (position, orientation) = odom_list.lookupTransform('map','base_footprint', rospy.Time(0))
    pose.position.x=position[0]
    pose.position.y=position[1]
    # the previous 2 lines and next 2 lines are repedative. Joes bad
    xPosition=position[0]
    yPosition=position[1]

    odomW = orientation
    q = [odomW[0], odomW[1], odomW[2], odomW[3]]
    roll, pitch, yaw = euler_from_quaternion(q)
    #convert yaw to degrees
    pose.orientation.z = yaw
    theta = yaw



# This is the program's main function
if __name__ == '__main__':
    rospy.init_node('Joes_Lab_2_example')
    global pub
    global pose
    global odom_list
    #global odom_tf
    pose = Pose()
    pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist,None, queue_size=10) # Publisher for commanding robot motion
    bumper_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, readBumper, queue_size=1) # Callback function to handle bumper events
    goal_sub = rospy.Subscriber('/move_base_simple/goal_cos', PoseStamped, navToPose, queue_size=1)

    rospy.Timer(rospy.Duration(.01), tCallback) # timer callback for robot location
    
    odom_list = tf.TransformListener() #listner for robot location

    rospy.sleep(2)

    print "Starting Lab 2"	
    while not rospy.is_shutdown():
        #publishTwist(10,0)
        #print "running"
        #rospy.sleep(0.5)
        rospy.spin()
    
    print "Lab 2 complete!"
