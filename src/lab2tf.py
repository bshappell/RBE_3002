#!/usr/bin/env python

import rospy, tf
from kobuki_msgs.msg import BumperEvent
# Add additional imports for each of the message types used

x =
#drive to a goal subscribed as /move_base_simple/goal
def navToPose(goal):

    print "spin!"

    print "move!"
    
	print "spin!"
    
	print "done"
	pass


#This function sequentially calls methods to perform a trajectory.
def executeTrajectory():
    pass  # Delete this 'pass' once implemented




#This function accepts two wheel velocities and a time interval.
def spinWheels(u1, u2, time):
    pass  # Delete this 'pass' once implemented



#This function accepts a speed and a distance for the robot to move in a straight line
def driveStraight(speed, distance):
    pass  # Delete this 'pass' once implemented


    
#Accepts an angle and makes the robot rotate around it.
def rotate(angle):
    pass  # Delete this 'pass' once implemented



#This function works the same as rotate how ever it does not publish linear velocities.
def driveArc(radius, speed, angle):
    pass  # Delete this 'pass' once implemented





#Bumper Event Callback function
def readBumper(msg):
    if (msg.state == 1):
        # What should happen when the bumper is pressed?
        pass  # Delete this 'pass' once implemented



# (Optional) If you need something to happen repeatedly at a fixed interval, write the code here.
# Start the timer with the following line of code: 
#   rospy.Timer(rospy.Duration(.01), timerCallback)
def getOdom(event):
    global pose
    pose = Pose()
    odom_list.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(1.0))
    (position, orientation) = odom_list.lookupTransform('map','base_footprint'', rospy.Time(0))
    x=position.pose.pose.position.x
    y=position.pose.pose.position.y
    odomW = orientation.pose.pose.orientation.x
    q = [odomW.x, odomW.y, odomW.z, odomW.w]
    roll, pitch, yaw = euler_from_quaternion(q)
    #convert yaw to degrees
    global theta
    theta = math.degrees(yaw)

	
    pass # Delete this 'pass' once implemented




def timerCallback():
# do somthing in 1 second intervals
    print 'Timer called at ' + str(event.current_real)



# This is the program's main function
if __name__ == '__main__':
    # Change this node name to include your username
    rospy.init_node('sample_Lab_2_node')


    # These are global variables. Write "global <variable_name>" in any other function
    #  to gain access to these global variables
    
    global pub
    global pose
    global odom_tf
    global odom_list

    
    # Replace the elipses '...' in the following lines to set up the publishers and subscribers the lab requires
    pub = rospy.Publisher('...', ...) # Publisher for commanding robot motion
    bumper_sub = rospy.Subscriber('...', ..., readBumper, queue_size=1) # Callback function to handle bumper events

    # Use this object to get the robot's Odometry 
    odom_list = tf.TransformListener()
    
    # Use this command to make the program wait for some seconds
    rospy.sleep(rospy.Duration(1, 0))
    rospy.Timer(rospy.Duration(1), timerCallback)


    print "Starting Lab 2"
    
    #make the robot keep doing something...
    

    # Make the robot do stuff...

    print "Lab 2 complete!"

