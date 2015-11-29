#!/usr/bin/env python

import rospy
from nav_msgs.msg import GridCells
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry, OccupancyGrid
from kobuki_msgs.msg import BumperEvent
import tf
import numpy
import math 




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

def publishCells(grid,num):
    global pub
    global ckd
    global front
    print "publishing"
    k=0
    cells = GridCells()
    cells.header.frame_id = 'map'
    cells.cell_width = 0.3 # edit for grid size .3 for simple map
    cells.cell_height = 0.3 # edit for grid size

    for i in range(1,height): #height should be set to hieght of grid
        for j in range(1,width): #height should be set to hieght of grid
            #print k # used for debugging
            if (grid[k] == num):
                point=Point()
                point.x=j*.3+.32 # edit for grid size
                point.y=i*.3-.15 # edit for grid size
                point.z=0
                cells.cells.append(point)
            k=k+1
        k=k+1
        if (grid[k] == num):
            point=Point()
            point.x=j*.3+.62 # edit for grid size
            point.y=i*.3-.15 # edit for grid size
            point.z=0
            cells.cells.append(point)

    #print cells # used for debugging
    if(num == 100):
        pub.publish(cells)           
    if(num == 1):
        ckd.publish(cells)
    if(num == 2):
        front.publish(cells)

def publishFront(grid):
    global front
    print "publishing"
    k=0
    cells = GridCells()
    cells.header.frame_id = 'map'
    cells.cell_width = 0.3 # edit for grid size .3 for simple map
    cells.cell_height = 0.3 # edit for grid size

    for i in range(1,10): #height should be set to hieght of grid
        for j in range(1,9): #height should be set to hieght of grid
            #print k # used for debugging
            if (grid[k] == 1):
                point=Point()
                point.x=j*.3+.32 # edit for grid size
                point.y=i*.3-.15 # edit for grid size
                point.z=0
                cells.cells.append(point)
            k=k+1
        k=k+1
        if (grid[k] == 1):
            point=Point()
            point.x=j*.3+.62 # edit for grid size
            point.y=i*.3-.15 # edit for grid size
            point.z=0
            cells.cells.append(point)

    #print cells # used for debugging
    front.publish(cells)           

def publishChecked(grid):
    global ckd
    print "publishing"
    k=0
    cells = GridCells()
    cells.header.frame_id = 'map'
    cells.cell_width = 0.3 # edit for grid size .3 for simple map
    cells.cell_height = 0.3 # edit for grid size

    for i in range(1,10): #height should be set to hieght of grid
        for j in range(1,9): #height should be set to hieght of grid
            #print k # used for debugging
            if (grid[k] == 0):
                point=Point()
                point.x=j*.3+.32 # edit for grid size
                point.y=i*.3-.15 # edit for grid size
                point.z=0
                cells.cells.append(point)
            k=k+1
        k=k+1
        if (grid[k] == 0):
            point=Point()
            point.x=j*.3+.62 # edit for grid size
            point.y=i*.3-.15 # edit for grid size
            point.z=0
            cells.cells.append(point)

    #print cells # used for debugging
    ckd.publish(cells)           


#Main handler of the project
def run():
    global pub
    global ckd
    global front
    rospy.init_node('lab3')
    sub = rospy.Subscriber("/map", OccupancyGrid, mapCallBack)
    pub = rospy.Publisher("/grid_walls", GridCells, queue_size=1)
    ckd = rospy.Publisher("/grid_checked", GridCells, queue_size=1)
    front = rospy.Publisher("/grid_Front", GridCells, queue_size=1)
    sleeper = rospy.Duration(1)
    rospy.sleep(sleeper)




    #used to make checker board # used for debugging
    checker=[1.0,1.0,1.0,1.0,1.0,0.0,1.0,0.0,1.0,0.0,
1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,
1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,
1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,
1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,
1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,
1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,
1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,
1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,
1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0]

    while (1 and not rospy.is_shutdown()):
        publishCells(mapData,100) #publishing map data every 2 seconds
        publishFront(checker)
        publishChecked(checker)
        for i in range(1, 100):
            if(mapData[i] == 1):
                print mapData[i]
        #        mapData[i] = 0
            else:
                print mapData[i]
        #        mapData[i] = 1
        rospy.sleep(2)  
        print("Complete")
    


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass

