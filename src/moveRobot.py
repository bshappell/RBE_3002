#!/usr/bin/env python

# Author: Breanne Happell
# RBE 3002 Lab 4

import rospy, tf
from kobuki_msgs.msg import BumperEvent

# Add additional imports for each of the message types used
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import BumperEvent
from tf.transformations import euler_from_quaternion
import tf
import numpy
import math 
import time

#Kobuki Dimensions
wheel_radius  = .035  #m
wheel_base = .230 #m

#Bumper State
bumperState = 0;


#drive to a goal subscribed as /move_base_simple/goal
def navToPose(goal):

    global pose 	
    global xPos # current x position
    global yPos # current y position
    global theta # initial theta
    global goal_x # goal x position
    global goal_y # goal y position
    global goal_theta # goal orientation

    #goal_x = goal_x - 2 # when in simulation
    #goal_y = goal_y - 2

    # determine angle to spin atan2(y,x)
    delta_x = goal_x - xPos
    delta_y = goal_y - yPos
    drive_theta1 = math.atan2(delta_y, delta_x)
    drive_theta = -(theta - drive_theta1)
    print "spin!"
    rotate(drive_theta)
    
    # determine distance to move
    dist = math.sqrt((delta_x)**2 + (delta_y)**2) #Distance formula
    print "move!"  
    driveStraight(0.1, dist)

    # rotate the differences in theta
    turnAngle = goal_theta - theta
    print "spin!"
    rotate(turnAngle)

    print "done"

#This function accepts a linear velocity and angular velocity and creates and publishes a twist message
def publishTwist(lin_vel, ang_vel):
    global pub

    twist_msg = Twist();		#Create Twist Message

    twist_msg.linear.x = lin_vel	#Populate message with data
    twist_msg.angular.z = ang_vel
    pub.publish(twist_msg)		#Send Message

#This function accepts two wheel velocities and a time interval.
def spinWheels(u1, u2, timesec):

    lin_vel = (wheel_radius/2)*(u1 + u2)
    ang_vel = (wheel_radius/wheel_base)*(u1-u2)
    
    # for a specified amount of time send twist message
    start = time.time()
    while(time.time() - start < timesec):
	    publishTwist(lin_vel, ang_vel)

#Odometry Callback function.
def readOdom(msg):
    global pose
    global odom_tf

    pose = msg.pose
    geo_quat = pose.pose.orientation
  
    #odom_tf.sendTransform((pose.pose.position.x, pose.pose.position.y, 0),
       # (pose.pose.orientation.x, pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w),rospy.Time.now(),"base_footprint","odom")

    px = msg.pose.pose.position.x
    py = msg.pose.pose.position.y
    quat = msg.pose.pose.orientation
    q = [quat.x, quat.y, quat.z, quat.w]
    roll, pitch, yaw = euler_from_quaternion(q)
    global xPos
    global yPos
    global theta
    xPos = px
    yPos = py
    xPos = int(xPos * 5)
    yPos = int(yPos * 5)
    #theta = math.degrees(yaw)
    theta = yaw



def getWayPoint(msg):

    print("received new waypoint")

    px = msg.pose.position.x
    py = msg.pose.position.y
    quat = msg.pose.orientation
    q = [quat.x, quat.y, quat.z, quat.w]
    roll, pitch, yaw = euler_from_quaternion(q)
    global goal_x
    global goal_y
    global goal_theta
    goal_x = px
    goal_y = py
    goal_theta = yaw    

    navToPose(msg)


#getOdomData: creates the subscriber for the odometry data
def getOdomData():
    sub = rospy.Subscriber("/odom", Odometry, odomCallback)


#This function accepts a speed and a distance for the robot to move in a straight line
def driveStraight(speed, distance):
    global odom_list
    global pose 	

    x0 = pose.pose.position.x	#Set origin
    y0 = pose.pose.position.y

    #Loop until the distance between the attached frame and the origin is equal to the
    #distance specifyed 
    done = False
    while (not done and not rospy.is_shutdown()):
        x1 = pose.pose.position.x
        y1 = pose.pose.position.y
        d = math.sqrt((x1-x0)**2 + (y1 - y0)**2) #Distance formula
        if (d >= distance):
            done = True
            publishTwist(0, 0)
        elif(d >= distance/2):
            publishTwist(speed/2,0)
        else:
            publishTwist(speed, 0)

    
#Accepts an angle and makes the robot rotate around it.
def rotate(angle):
    print("entered rotate")
    global odom_list
    global pose

    #This node was created using Coordinate system transforms and numpy arrays.
    #The goal is measured in the turtlebot's frame, transformed to the odom.frame 
    transformer = tf.TransformerROS()	
    rotation = numpy.array([[math.cos(angle), -math.sin(angle), 0],	#Create goal rotation
                            [math.sin(angle), math.cos(angle), 0],
                            [0,          0,          1]])

    #Get transforms for frames
    odom_list.waitForTransform('odom', 'base_footprint', rospy.Time(0), rospy.Duration(4.0))
    (trans, rot) = odom_list.lookupTransform('odom', 'base_footprint', rospy.Time(0))
    T_o_t = transformer.fromTranslationRotation(trans, rot)
    R_o_t = T_o_t[0:3,0:3]

    #Setup goal matrix
    goal_rot = numpy.dot(rotation, R_o_t)
    goal_o = numpy.array([[goal_rot[0,0], goal_rot[0,1], goal_rot[0,2], T_o_t[0,3]],
                    [goal_rot[1,0], goal_rot[1,1], goal_rot[1,2], T_o_t[1,3]],
                    [goal_rot[2,0], goal_rot[2,1], goal_rot[2,2], T_o_t[2,3]],
                    [0,             0,             0,             1]])

    #Continues creating and matching coordinate transforms.
    done = False
    print("entering rotate while loop")
    while (not done and not rospy.is_shutdown()):
        (trans, rot) = odom_list.lookupTransform('odom', 'base_footprint', rospy.Time(0))
        state = transformer.fromTranslationRotation(trans, rot)
        within_tolerance = abs((state - goal_o)) < .2
        if ( within_tolerance.all() ):
            spinWheels(0,0,0)
            done = True
        else:
            if (angle > 0):
                spinWheels(2,-2,.1)
            else:
                spinWheels(-2,2,.1)


#Bumper Event Callback function
def readBumper(msg):
    if (msg.state == 1):
        # What should happen when the bumper is pressed?
        bumperState = 1 
    else:
        bumperState = 0
        executeTrajectory() 


# This is the program's main function
if __name__ == '__main__':
    # New node to handle moving the robot
    rospy.init_node('Lab4MoveRobot')
    
    global pub
    global pose
    global odom_tf
    global odom_list
    
    pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1) # Publisher for commanding robot motion
    #bumper_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, readBumper, queue_size=1) # Callback function to handle bumper events

    # Use this object to get the robot's Odometry 
    sub = rospy.Subscriber('/odom', Odometry, readOdom)
    odom_list = tf.TransformListener()
    odom_tf = tf.TransformBroadcaster()
    odom_tf.sendTransform((0, 0, 0),(0, 0, 0, 1),rospy.Time.now(),"base_footprint","odom")

    # rviz clicker subscriber
    #rviz_sub = rospy.Subscriber('not/move_base_simple/goal', PoseStamped, getGoal)
    wayPointSub = rospy.Subscriber('waypoint', PoseStamped, getWayPoint)

    # Use this command to make the program wait for some seconds
    #rospy.sleep(rospy.Duration(1, 0))



    print "Starting Lab 4 Move Robot"

    while not rospy.is_shutdown():
        print("starting")
        rospy.spin() 

    print "Lab 2 complete!"

