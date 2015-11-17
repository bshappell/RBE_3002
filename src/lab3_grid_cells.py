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

def publishCells(grid):
    global pub
    print "publishing"
    k=0
    cells = GridCells()
    cells.header.frame_id = 'map'
    cells.cell_width = 0.3 # edit for grid size .3 for simple map
    cells.cell_height = 0.3 # edit for grid size

    for i in range(1,height): #height should be set to hieght of grid
        k=k+1
        for j in range(1,width): #height should be set to hieght of grid
            k=k+1
            #print k # used for debugging
            if (grid[k] == 100):
                point=Point()
                point.x=j*.3 # edit for grid size
                point.y=i*.3 # edit for grid size
                point.z=0
                cells.cells.append(point)

    #print cells # used for debugging
    pub.publish(cells)           

#Main handler of the project
def run():
    global pub
    rospy.init_node('lab3')
    sub = rospy.Subscriber("/map", OccupancyGrid, mapCallBack)
    pub = rospy.Publisher("/grid_check", GridCells, queue_size=1)
    sleeper = rospy.Duration(1)
    rospy.sleep(sleeper)




    #used to make checker board # used for debugging
    checker=[1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,
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
        publishCells(mapData) #publishing map data every 2 seconds
        rospy.sleep(2)  
        print("Complete")
    


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
