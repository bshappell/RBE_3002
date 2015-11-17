#!/usr/bin/env python

import rospy
import AStar
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


# reads in map data
def mapCallBack(data):
    global mapData, grid
    global width
    global height
    global mapgrid
    mapgrid = data
    mapData = data.data
    width = data.info.width
    height = data.info.height
    #print data.info # used for debugging
    #publishCells(mapgrid.data) # used for debugging
	
def publishCells(grid):
    global wallpub
    print "publishing"
    k=0
    cells = GridCells()
    cells.header.frame_id = 'map'
    cells.cell_width = 0.3 # edit for grid size .3 for simple map
    cells.cell_height = 0.3 # edit for grid size

    for i in range(1,height): #height should be set to hieght of grid
        for j in range(1,width): #height should be set to hieght of grid
            #print k # used for debugging
            if (grid[k] == 100):
                point=Point()
                point.x=j*.3+.32 # edit for grid size
                point.y=i*.3-.15 # edit for grid size
                point.z=0
                cells.cells.append(point)
            k=k+1
        k=k+1
        if (grid[k] == 100):
            point=Point()
            point.x=j*.3+.62 # edit for grid size
            point.y=i*.3-.15 # edit for grid size
            point.z=0
            cells.cells.append(point)

    #print cells # used for debugging
    wallpub.publish(cells)           
	

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
    xInit = 0
    yInit = 0
    xEnd = 10
    yEnd = 10
    width = 10
    height = 10

    AMap = 0
    worldMap = 0
    path = 0

    rospy.init_node('lab3')
    worldMapSub = rospy.Subscriber('/map', OccupancyGrid, readWorldMap)
    markerSub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, readGoal)
    sub = rospy.Subscriber("/initialPose", PoseWithCovarianceStamped, startCallBack)
    mapsub = rospy.Subscriber("/map", OccupancyGrid, mapCallBack)
    wallpub = rospy.Publisher("/grid_walls", GridCells, queue_size=1)
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


    while not rospy.is_shutdown():
        print("starting")
        rospy.sleep(1)
        publishCells(mapData) #publishing map data every 2 seconds
        AStar(xInit, yInit, xEnd, yEnd, width, height)
        print("complete")
        rospy.loginfo("Complete")
        rospy.spin() 
















   
