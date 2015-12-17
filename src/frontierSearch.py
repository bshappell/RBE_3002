#!/usr/bin/env python

import math 
import numpy
import rospy
import Queue
from nav_msgs.msg import OccupancyGrid, GridCells, Path, Odometry
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Quaternion, _Quaternion
from kobuki_msgs.msg import BumperEvent
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped
import tf
import numpy
import math 
import copy

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y

# runs frontier search until all nodes have been explored
def runFrontierSearch():

    global xOdom
    global yOdom
    global xOffset
    global yOffset
    global res
    global frontierCells
    global roboPose_x
    global roboPose_y
    global wallList
    global clearList

    # convert odometry
    roboPose_x = (xOdom - xOffset) / res
    roboPose_y = (yOdom - yOffset) / res
    
    print "LENGTH OF FRONTIER LIST: ", len(frontierCells)
    print "LENGTH OF WALL LIST: ", len(wallList)
    print "LENGTH OF CLEAR LIST: ", len(clearList)

    # publish the wall and clear cells
    publishCells(wallList,2)
    publishCells(clearList,1)

    # continue running until all of the frontier is explored
    #while(len(frontierCells) and not rospy.is_shutdown()):
    
    print "robo Pose x: ", roboPose_x, " roboPose_y: ", roboPose_y

    frontierSearch(roboPose_x, roboPose_y)


def frontierSearch(curr_x, curr_y):    

    global width
    global height
    global frontierCells

    bfsFrontier = [] # store the nodes to be expanded
    frontier = [] # store the nodes in the frontier of the robot's map
    checked = [] # stores all checked nodes that are not on the bfs frontier

    # make the initial starting position node
    curr_node = Node(curr_x, curr_y)
    bfsFrontier.append(curr_node) # add the node to the BFS frontier
    

    # BFS to find all frontier nodes
    while(len(bfsFrontier) and not rospy.is_shutdown()):
        print "enters while loop (step 0)"

        # get first node from the bfs frontier list
        curr_node = bfsFrontier.pop(0)

        # add current node to the checked list
        checked.append(curr_node)

        #get a list of all neighboring x and y coordiantes 
        neighbors = getNeighbors(curr_node.x, curr_node.y) 

        # iterate through list of neighbors
        for i in range(4):
            new_x = neighbors[i][0] 
            new_y = neighbors[i][1]

            # check that the node has not been checked yet
            if(not listContains(checked, new_x, new_y)):

                print "step 1"
            
                #if in bounds (equal to zero less than width)
                if 0 <= new_x and new_x < width and 0 <= new_y and new_y < height:
                    print "in bounds (step 2)"

                    # make a new node
                    node = Node(new_x, new_y)

                    # add it to the checked list
                    checked.append(node)

                    # if it is not a wall or an unknown value
                    if((not getWallVal(new_x, new_y)) and (getClearVal(new_x, new_y))):
                        print " appropriate wall val (step 3)"
                        
                        # add the node to the BFS frontier
                        bfsFrontier.append(node)
                        
                        # determine if node is on the frontier
                        if(onFrontier(node)):
                            print "ON FRONTIERRRRR (step 4)"
                            # add to the frontier list
                            frontier.append(node)

        
    # when list of all frontier nodes is calculated determine the centroids of each cluster
    #determineCentroids(frontierList)
    for item in frontier:
        print "frontier x: ", item.x, " frontier y: ", item.y

    # publish frontier cells to the map
    publishCells(frontier, 0)

    # set global variable equal to current frontier list
    frontierCells = copy.copy(frontier)

    # determine the centroids for the frontier list
    determineCentroids(frontier)


# returns true if a given node is considered a frontier on the robot map
def onFrontier(node):

    # is a frontier if it has been explored and is not a wall and has an unexplored neighbor

    hasUnexploredNeighbor = False # initailly there is no known unexplored neighbor

    print " testing x: ", node.x, " testing y: ", node.y

    # check that the node is clear
    if(getClearVal(node.x, node.y)):
        print "Is clear val"
        
        #get a list of all neighboring x and y coordiantes 
        neighbors = getNeighbors(node.x, node.y) 

        # iterate through list of neighbors
        for i in range(4):
            new_x = neighbors[i][0] 
            new_y = neighbors[i][1]

            # node is considered unexplored is it is not a wall or clear
            if(not getWallVal(new_x, new_y) and (not getClearVal(new_x, new_y))):
                return True # if so return true

    return False # else not a frontier node, return false


# takes in a list of all frontier nodes and calculate the centroids
def determineCentroids(frontierList):

    centroidList = [] # list of all possible centroids
    bfsFrontier = [] # stores the frontier for the BFS

    # continue searching until frontier list is empty
    while(len(frontierList) and not rospy.is_shutdown()):

        clusterList = [] # stores all of the nodes on the current cluster

        # remove the first node from the frontier list and add it to the current cluster list
        node = frontierList.pop(0)
        clusterList.append(node)

        # add the node to the bfs frontier
        bfsFrontier.append(node)

        # continue searching until BFS frontier is empty
        while(len(bfsFrontier)): #and not rospy.is_shutdown()):

            # take first node from bfs frontier list
            node = bfsFrontier.pop(0)

            #get a list of all neighboring x and y coordiantes 
            neighbors = getNeighbors(node.x, node.y) 

            # iterate through list of neighbors
            for i in range(4):
                new_x = neighbors[i][0] 
                new_y = neighbors[i][1]

                # determine if neighbor is on the frontier list
                index = getIndexPlace(frontierList, new_x, new_y)

                # if index is greater than -10 the node exists in the list
                if(index > -10):

                    # remove node from the frontier list and add to the current cluster list
                    curr_node = frontierList.pop(index)
                    clusterList.append(curr_node)

                    # add node to the bfs frontier list
                    bfsFrontier.append(curr_node)
 

        print "cluster list: "
        for i in clusterList:
            print "x: ", i.x, " y: ", i.y

        # append centroid of cluster to centroid list
        centroidList.append(calculateCentroid(clusterList))

        # send the robot to the closest frontier 
        sendGoal(centroidList)


# takes in the list of frontier centroids and sends the robot to the the closest one
def sendGoal(centroidList):
    
    global xOdom
    global yOdom
    global roboPose_x
    global roboPose_y
    
    closestX = 0
    closestY = 0
    minDistance = 1000

    # loop through the list of centroids and sen
    for centroid in centroidList:
        
        # determine if its closer than the current closest centroid
        if(math.sqrt((centroid.x - roboPose_x)**2 + (centroid.y - roboPose_y)**2) < minDistance):

            # update the closest centroid variables
            closestX = centroid.x
            closestY = centroid.y
            minDistance = math.sqrt((centroid.x - roboPose_x)**2 + (centroid.y - roboPose_y)**2)
     
    print "PUBLISH GOAL: X: ", closestX, " Y: ", closestY
    # call publish goal
    publishGoal(closestX, closestY)

#publish next way point as poseStamped
def publishGoal(xPos, yPos):
    
    global moveRobot
    global xOffset
    global yOffset
    global res

    goalX = float(xPos * res) + xOffset
    goalY = float(yPos * res) + yOffset
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.pose.position.x = goalX
    goal.pose.position.y = goalY
    goal.pose.position.z = 0
    (w, x, y, z) = quaternion_from_euler(0, 0, 0)
    goal.pose.orientation = Quaternion(w, x, y, z)
    moveRobot.publish(goal)

# takes in a list of a cluster of nodes and returns the centroid for that list
def calculateCentroid(clusterList):

    xSum = 0 # initialize sums to zero
    ySum = 0

    # loop through each node in the cluster list
    for node in clusterList:

        # add the x and y values to the sum
        xSum = xSum + node.x
        ySum = ySum + node.y

    # determine the coordinates of the centroid
    centroid_x = int(xSum/len(clusterList))
    centroid_y = int(ySum/len(clusterList))

    # doulble check that the centroid is not a wall
    if(not getWallVal(centroid_x, centroid_y)):
        print "centroid x: ", centroid_x, " centroid y: ", centroid_y
        return Node(centroid_x, centroid_y)
    # else just return the first value of the clusterList
    else:
        return clusterList.pop(0)


# gets the wall value for the given x and y coordinates and returns the correct value
def getWallVal(xVal, yVal):
    global wallList
    #print wallList
    for val in wallList:
        if(val.x == xVal and val.y == yVal):
            return True
    return False

# returns true if it has a "wall val of zero" and false if its a wall or unknown
def getClearVal(xVal, yVal):
    global clearList
    for val in clearList:
        if(val.x == xVal and val.y == yVal):
            return True
    return False

#returns a list of the tuples for all the neighbor x and y values
def getNeighbors(curr_x, curr_y):

    neighbors = []
	
    #check neighbor to north
    delta_x = [0,1,0,-1]
    delta_y = [1,0,-1,0]
    #print "neighbors x: ", curr_x, "neighbors y: ", curr_y
    for i in range(4):
        new_x = curr_x + delta_x[i]
        new_y = curr_y + delta_y[i]
        point = (new_x,new_y)
        neighbors.append(point)
        #print "neighbor_x ", new_x, "neighbor_y ",new_y
		
    return neighbors

# goes through a list and returns the idex for the desired cell
def getIndexPlace(listToSearch, xCoord, yCoord):

    for i in range(len(listToSearch)):
        listItem = listToSearch[i]
        if( xCoord == listItem.x and yCoord == listItem.y):
            return i
    print "list Item Not Found" 
    print "x: ", xCoord, " y: ", yCoord
    return -10

# returns true if a given list returns the given coordinates
def listContains(listToSearch, xCoord, yCoord):
    for i in range(len(listToSearch)):
        listItem = listToSearch[i]
        if( xCoord == listItem.x and yCoord == listItem.y):
            return True
    return False

def makeTestMap():

    global wallList
    global clearList
    global height
    global width
    height = 20
    width = 20
    wallList = []
    clearList = []

    wallList.append(Node(2,5))
    wallList.append(Node(2,4))

    # add a square of clear values around start
    for x in range(2,6):
        for y in range(2,8):
            curr_tuple = Node(x,y)
            print "clear tuplex: ", curr_tuple.x , "clear tupleY: ", curr_tuple.y

            clearList.append(curr_tuple)

    testList = []
    node = Node(3,3)
    testList.append(node)
    node = Node(3,4)
    testList.append(node)
    node = Node(3,5)
    testList.append(node)
    node = Node(3,6)
    testList.append(node)
    node = Node(3,7)
    testList.append(node)
    node = Node(8,8)
    testList.append(node)
    node = Node(9,8)
    testList.append(node)
    node = Node(9,9)
    testList.append(node)
    

    determineCentroids(testList)
    

#Odometry Callback function.
def readOdom(msg):
    global pose
    global xOdom
    global yOdom
    global theta
    global width
    global height
    # odom list wait for transform
    odom_list.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(1.0))
    (position, orientation) = odom_list.lookupTransform('map', 'base_footprint', rospy.Time(0))
    xOdom = position[0]
    yOdom = position[1]
    odomW = orientation
    q = [odomW[0], odomW[1], odomW[2], odomW[3]]
    roll,pitch,yaw = euler_from_quaternion(q)
    theta = yaw


# reads in map data
def mapCallBack(data):
    global mapData, grid
    global width
    global height
    global mapgrid
    global xOffset
    global yOffset
    global res
    print "MAPCALLBACK"
    mapgrid = data
    mapData = data.data
    width = data.info.width
    height = data.info.height
    res = data.info.resolution
    xOffset = data.info.origin.position.x
    yOffset = data.info.origin.position.y
    publishWalls(mapgrid.data, res) # used for debugging
    #print getWallVal(0,0)


def publishWalls(grid,res):
    #print "publishing"
    global wallpub
    global width
    global height
    global xOffset
    global yOffset
    global wallList
    global clearList
    k=0
    cells = GridCells()
    cells.header.frame_id = 'map'
    cells.cell_width = res # edit for grid size .3 for simple map
    cells.cell_height = res # edit for grid size

    for i in range(1,height): #height should be set to hieght of grid
        for j in range(1,width): #height should be set to hieght of grid
            #print k # used for debugging
            if (grid[k] == 100):
                #print "HERE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                point=Point()
                point.x=j*cells.cell_width+xOffset # edit for grid size
                point.y=i*cells.cell_height+yOffset # edit for grid size
                point.z=0
                wallList.append(Node(j-1,i-1))
            elif (grid[k] == 0):
                #print "CLEAR!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                point=Point()
                point.x=j*cells.cell_width+xOffset # edit for grid size
                point.y=i*cells.cell_height+yOffset # edit for grid size
                point.z=0
                clearList.append(Node(j-1,i-1))
            k=k+1
        k=k+1

        set(wallList)
        set(clearList)

    #print cells # used for debugging
    print "mapped"
    #wallPub.publish(cells) 
    



# takes in a list of nodes
#grid is the list of points that have been checked
# 0 = frontier cells, 1 = clear cells, 2 = wall cells
def publishCells(grid, num):

    global frontierPub
    global clearPub
    global wallPub
    global xOffset
    global yOffset
    print "publishing"
    k=0
    cells = GridCells()
    cells.header.frame_id = 'map'
    cells.cell_width = 0.2 # edit for grid size .3 for simple map
    cells.cell_height = 0.2 # edit for grid size

    for square in grid: #height should be set to hieght of grid
            #print k # used for debugging
            point=Point()
            point.x=square.x*cells.cell_width+cells.cell_width+xOffset # edit for grid size
            point.y=square.y*cells.cell_height+cells.cell_width+yOffset # edit for grid size
            point.z=0
            cells.cells.append(point)

    #print cells # used for debugging
    if(num == 0):    
        frontierPub.publish(cells)
    elif(num == 1):    
        clearPub.publish(cells)
    elif(num == 2):    
        wallPub.publish(cells)


# main function of program
if __name__ == '__main__':

    rospy.init_node('bshappell_kcorton_nkjefferson_frontierSearch')

    global frontierPub
    global moveRobot
    global mapSub
    global xOdom
    global yOdom
    global wallList
    global clearList
    global frontierCells
    global roboPose_x
    global roboPose_y
    global clearPub
    global wallPub

    # intialize global variables
    xOdom = 0
    yOdom = 0
    xOffset = 0
    yOffset = 0
    width = 10
    height = 10
    roboPose_x = 0
    roboPose_y = 0
    wallList = []
    clearList = []
    frontierCells = []
    frontierCells.append(Node(0,0))

    # publish frontier cells to the map
    frontierPub = rospy.Publisher("/grid_frontier", GridCells, queue_size=1)

    # publish clear cells to the map
    clearPub = rospy.Publisher("/grid_frontier_clear", GridCells, queue_size=1)

    # publish wall cells to the map
    wallPub = rospy.Publisher("/grid_frontier_walls", GridCells, queue_size=1)

    # publisher to publish robot driving position
    moveRobot = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)

    # map subscriber
    mapsub = rospy.Subscriber('/finished', OccupancyGrid, mapCallBack)

    # get robot odometry
    odom_list = tf.TransformListener()
    rospy.sleep(rospy.Duration(1,0))
    rospy.Timer(rospy.Duration(0.1), readOdom) 

    

    #makeTestMap()

    while 1 and not rospy.is_shutdown():
        print("starting")
        rospy.sleep(1)
        runFrontierSearch()
        print("completed frontier search!!!!")
        rospy.loginfo("Complete")

