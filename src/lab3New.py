#!/usr/bin/env python

import rospy
import AStarNew
import roslib
import time
import math
from numpy import *
from nav_msgs.msg import OccupancyGrid, GridCells, Path, Odometry
from geometry_msgs.msg import Point, Pose, PoseStamped, Twist, PoseWithCovarianceStamped, Quaternion
from AStar import AStar
import time
from tf.transformations import euler_from_quaternion

xInit = 0
yInit = 0
thetaInit = 0

xEnd = 0
yEnd = 0
thetaEnd = 0


def readWorldMap(data):
# map listener
    global mapData, grid
    global width
    global height
    grid = data
    mapData = data.data
    width = data.info.width
    height = data.info.height

def readGoal(msg):
    px = msg.pose.position.x
    py = msg.pose.position.y
    quat = msg.pose.orientation
    q = [quat.x, quat.y, quat.z, quat.w]
    roll, pitch, yaw = euler_from_quaternion(q)
    global xEnd
    global yEnd
    global thetaEnd
    xEnd = px
    yEnd = py
    thetaEnd = yaw * 180.0 / math.pi

	
def startCallBack(data):
    px = data.pose.pose.position.x
    py = data.pose.pose.position.y
    quat = data.pose.pose.orientation
    q = [quat.x, quat.y, quat.z, quat.w]
    roll, pitch, yaw = euler_from_quaternion(q)
    global xInit
    global yInit
    global thetaInit
    xInit = px
    yInit = py
    thetaInit = yaw * 180.0 / math.pi


	

if __name__ == '__main__':

    global worldMap
    global target
    global xInit
    global yInit
    global xEnd
    global yEnd
    global width
    global height
    global wallpub
    global xPosPose
    global yPosPose
    xInit = 4
    yInit = 6
    xEnd = 27
    yEnd = 9
    width = 30
    height = 30

    AMap = 0
    worldMap = 0
    path = 0

    rospy.init_node('bshappell_kcorton_nkjefferson_lab4')
    worldMapSub = rospy.Subscriber('/map', OccupancyGrid, readWorldMap)
    markerSub = rospy.Subscriber('/move_base_simple/goal1', PoseStamped, readGoal)
    sub = rospy.Subscriber("/initialPose1", PoseWithCovarianceStamped, startCallBack)
    cellPub = rospy.Publisher('/cell_path', GridCells, queue_size=1)
    pathPub = rospy.Publisher('/path_path', Path, queue_size=1)
    

    target = 0
    start = 0
    end = 0



    # resolution and offset of the map

    # create a new instance of the map

    # generate a path to the start and end goals

    # for each node in the path, process the nodes to generate GridCells and Path messages
  
    # transform coordinates for map resolution and offset

    # continue making messages

    # do not stop publishing

    # initialize astar publishes and subscribers
    AStarNew.initAstar()


    while 1 and not rospy.is_shutdown():
        print("starting")
        rospy.sleep(1)
        print "start pose"
        print xInit
        print yInit
        print "end pose"
        print xEnd
        print yEnd
        AStarNew.getNextWayPoint(width, height)
        #print "goal reached!!!"
        #print xEnd
        #print yEnd
        print("complete")
        rospy.loginfo("Complete")
        #rospy.spin() 
















   
