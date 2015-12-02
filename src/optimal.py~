#!/usr/bin/env python

"""
Created, tested and maintained by Rohit for RBE 3002 Final Project
"""
import rospy
import copy
from nav_msgs.msg import OccupancyGrid

"""
The purpose of this node is to expand obstacles from the /map topic and
publish the new map to /map_OE. All references to OE stand for 
Obstacle Expansion, Obstacle Expanded or something along those lines. 
"""
class MapOpt:
    
    def beginOpt(self, map):
        
        map_resolution = round(map.info.resolution, 3)
        map_width = map.info.width
        map_height = map.info.height
        map_x_offset = round(map.info.origin.position.x + (0.5 * map_resolution), 1)
        map_y_offset = round(map.info.origin.position.y + (0.5 * map_resolution), 1)
    
        #for Occupancy Grid Optimization
        map_scale = int(round(self.robotResolution / map_resolution)) #scale factor
        new_map = copy.copy(map)
        
        new_map.info.width = int(round(map_width / map_scale, 0))
        new_map.info.height = int(round(map_height / map_scale, 0))
        new_map.info.resolution = self.robotResolution 
        
        map_scaled_width = new_map.info.width
        map_scaled_height = new_map.info.height
        map_scaled_cell_width = self.robotResolution
        map_scaled_cell_height = self.robotResolution
           
        MapPosStatus = {}
        for x in xrange(0, map_scaled_width):
            for y in xrange(0, map_scaled_height):
                index = y * map_scaled_width + x
                MapPosStatus[(x,y)] = 0
        xy = []
        for i in xrange(0, map_scale):
            for j in xrange(0, map_scale):
                xy.append((i,j));
        
        new_data = [0,] * (map_scaled_width * map_scaled_height)
        for x in xrange(0, map_scaled_width):
            for y in xrange(0, map_scaled_height):
                cost = 0
                nodes = 0
                for node in xy:
                    nodex = x * map_scale + node[0]
                    nodey = y * map_scale + node[1]
                    index = nodey * map_width + nodex
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
                new_data[y * map_scaled_width + x] = cost
                
        
        new_map.data = tuple(new_data)
        self.optmap_pub.publish(new_map)
        print "Done"
    
    def __init__(self, robotResolution = 0.2):
        # Initialize Node
        rospy.init_node('rbansal_vcunha_dbourque_MapOpt')
        
        # Setup publisher and Subscriber
        self.optmap_pub = rospy.Publisher('/rbefinal/map_Opt', OccupancyGrid, latch=True)
        self.OEmap_sub = rospy.Subscriber('/rbefinal/map_OE', OccupancyGrid, self.beginOpt, queue_size=1)
        
        # Store robot resolution (default is 0.2)
        self.robotResolution = robotResolution

        
# This is the program's main function
if __name__ == '__main__':
    
    # Modify this in case of a different robot resolution
    robotResolution = 0.2
    
    # Create MapOE object
    map_OPT = MapOpt(robotResolution)
    rospy.spin()
