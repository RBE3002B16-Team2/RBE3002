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

class Movement:

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


    def iteratePoses(positions):
        count = 0
        while(positions.size < count):
            navToPose(positions[count])
            count = count + 1
    
    
    
