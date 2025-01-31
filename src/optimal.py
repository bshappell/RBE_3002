
'''
Authored by bshappell, nkjefferson, kcorton
Submitted 12/17/2015
RBE 3002
Final Project
'''

import rospy
import copy
from nav_msgs.msg import OccupancyGrid


#The purpose of this file is to convert the occupied and mapped tiles from obstacle.py
#and reformat them to be a specific size and distance away form the object so that the robot is
#kept at a safe distance from the obstacle.

class MapOpt:
    
    def beginOpt(self, map):
        
        mapRes = round(map.info.resolution, 3)
        mapWidth = map.info.width
        mapHeight = map.info.height
        xOffset = round(map.info.origin.position.x + (0.5 * mapRes), 1)
        yOffset = round(map.info.origin.position.y + (0.5 * mapRes), 1)
    
        #for Occupancy Grid Optimization
        scale = int(round(self.robotResolution / mapRes)) #scale factor
	#copy the map so the data in it is not altered
        newMap = copy.copy(map)
        
        newMap.info.width = int(round(mapWidth / scale, 0))
        newMap.info.height = int(round(mapHeight / scale, 0))
        newMap.info.resolution = self.robotResolution 
        
        scaledWidth = newMap.info.width
        scaledHeight = newMap.info.height
        scaledCellWidth = self.robotResolution
        scaledCellHeight = self.robotResolution
           
        MapPosStatus = {}
        for x in xrange(0, scaledWidth):
            for y in xrange(0, scaledHeight):
                index = y * scaledWidth + x
                MapPosStatus[(x,y)] = 0
        xy = []
        for i in xrange(0, scale):
            for j in xrange(0, scale):
                xy.append((i,j));
        
        new_data = [0,] * (scaledWidth * scaledHeight)
        for x in xrange(0, scaledWidth):
            for y in xrange(0, scaledHeight):
                cost = 0
                nodes = 0
                for node in xy:
                    nodex = x * scale + node[0]
                    nodey = y * scale + node[1]
                    index = nodey * mapWidth + nodex
                    if(index >= 0 and index < len(map.data)):
                        if(map.data[index] > 0):
                            cost = cost + map.data[index]
                        elif(map.data[index] == 0):
                            cost = cost + 2
                        else:
                            cost = cost + -1
                        nodes = nodes+1
                cost = cost/nodes
                if(cost > 3 ):
                    cost = 100
                elif(cost < 0):
                    cost = -1
                else:
                    cost = 0
                new_data[y * scaledWidth + x] = cost
                
        
        newMap.data = tuple(new_data)
        self.optmap_pub.publish(newMap)
        print "Done"
    
    def __init__(self, robotResolution = 0.2):
        # Initialize Node
        rospy.init_node('optimizer')
        
        # Setup publisher and Subscriber
        self.optmap_pub = rospy.Publisher('/finished', OccupancyGrid, latch=True)
        self.OEmap_sub = rospy.Subscriber('/grid_walls', OccupancyGrid, self.beginOpt, queue_size=1)
        
        # Store robot resolution (default is 0.2)
        self.robotResolution = robotResolution

        
# This is the program's main function
if __name__ == '__main__':
    
    # Modify this in case of a different robot resolution
    robotResolution = 0.2
    
    # Create MapOE object
    map_OPT = MapOpt(robotResolution)
    rospy.spin()
