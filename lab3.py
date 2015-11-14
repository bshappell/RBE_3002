#!/usr/bin/env python

import rospy
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


def realWorldMap(data):
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

    AMap = 0
    worldMap = 0
    path = 0

    rospy.init_node('lab3')
    worldMapSub = rospy.Subscriber('/map', OccupancyGrid, readWorldMap)
    markerSub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, readGoal)
    sub = rospy.Subscriber("/initialPose", PoseWithCovarianceStamped, startCallBack)
    cellPub = rospy.Publisher('/cell_path', GridCells)
    pathPub = rospy.Publisher('/path_path', Path)

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
   
