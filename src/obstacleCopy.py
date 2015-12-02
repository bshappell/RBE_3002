#File to process obstacle expansion in the final map for frontier based
#searching

import rospy
import copy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import GridCells


# reads in map data
def mapCallBack(data):
    global mapData, grid
    global width
    global height
    global mapgrid
    global res
    mapgrid = data
    mapData = data.data
    width = data.info.width
    height = data.info.height
    res = data.info.resolution
    #print data.info # used for debugging
    print "data recieved"
    expandObstacles(mapgrid) # used for debugging

def expandObstacles(map):
    global res
    global width
    global height
    global botRadius
    global pub
    
    if(botRadius < res):
        eMap = map

    else:
        tarExp = int(round(botRadius/map.info.resolution,0))
        eMap = copy.copy(map)

        #organize the data into a managable list
        arrOfVals = {}
        for x in range(0, width):
            for y in range(0, height):
                index = y * width + x
                arrOfVals[(x,y)] = map.data[index]

        #create a grid appropiratly sized to resize the resolution of the 
        #map 
        resizeGrid = []
        for i in range(-tarExp, tarExp+1):
            for j in range(-tarExp, tarExp+1):
                resizeGrid.append((i,j))

        tempData = list(eMap.data)

        for x in range(0, width):
            for y in range(0,height):
                if(arrOfVals[x,y] == 100):
                    for element in resizeGrid:
                        index = (y + element[1]) * width + (x+element[0])
                        if(index >= 0 and index < len(tempData)):
                            tempData[index] = 100

        eMap.data = tuple(tempData)

    
    print "mapCallBack published!"
    pub.publish(eMap) 

def run():
    global pub
    global botRadius
    botRadius = .2
    rospy.init_node('expandedMap')
    sub = rospy.Subscriber("/map", OccupancyGrid, mapCallBack, queue_size=1)
    pub = rospy.Publisher("/grid_walls", OccupancyGrid, latch=True)
    sleeper = rospy.Duration(1)
    rospy.sleep(sleeper)




    while (1 and not rospy.is_shutdown()):
        #publishCells(mapData,100) #publishing map data every 2 seconds

        sleeper = rospy.Duration(1)
        rospy.sleep(sleeper)
     
if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
