#!/usr/bin/env python


import math 
import numpy
import rospy
import Queue
from actionlib_msgs.msg import GoalID, GoalStatusArray
from nav_msgs.msg import OccupancyGrid, GridCells, Path, Odometry
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Quaternion, _Quaternion
from kobuki_msgs.msg import BumperEvent
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped
import tf
import numpy
import math 

class GridSquare:
    def __init__(self, x, y, h, g, f, wallVal, cameFrom):
        self.x = x
        self.y = y
        self.h = h
        self.g = g
        self.f = f
        self.wallVal = wallVal
        self.cameFrom = cameFrom 

class WallSquare:
    def __init__(self, x, y, wallVal):
        self.x = x
        self.y = y
        self.wallVal = wallVal

#call back for move base status messages
def moveBaseStatus(msg):

    global activeGoal
    global goalReached
    global goalUnreachable
    global notInitStartup

    #print "move base status is called!"

    if notInitStartup:

        for goal in msg.status_list:
            print "msg Status", goal.status
            if goal.status < 2:
                print "active goal Found"
                activeGoal = goal
                goalReached = False 
    
        if len(msg.status_list) > 0 and not goalReached:
            for goal in msg.status_list:
                if goal.goal_id.id == activeGoal.goal_id.id:
                    if 2 <= goal.status <= 3:
                        goalReached = True
                    elif 4 <= goal.status <= 5: 
                        goalUnreachable = True
                        print "error goal unreachable in A*"


# reads in map data
def mapCallBack(data):
    global mapData, grid
    global width
    global height
    global mapgrid
    global xOffset
    global yOffset
    global res
    mapgrid = data
    mapData = data.data
    width = data.info.width
    height = data.info.height
    res = data.info.resolution
    xOffset = data.info.origin.position.x
    yOffset = data.info.origin.position.y
    #print data.info # used for debugging
    #print "mapped"
    #updateWallList(mapData) # update global wall value list
    publishWalls(mapgrid.data, res) # used for debugging
    print getWallVal(0,0)
	
def publishWalls(grid,res):
    #print "publishing"
    global wallpub
    global width
    global height
    global xOffset
    global yOffset
    global wallList
    k=0
    cells = GridCells()
    cells.header.frame_id = 'map'
    cells.cell_width = res # edit for grid size .3 for simple map
    cells.cell_height = res # edit for grid size

    for i in range(1,height): #height should be set to hieght of grid
        for j in range(1,width): #height should be set to hieght of grid
            #print k # used for debugging
            if (grid[k] == 100):
                point=Point()
                point.x=j*cells.cell_width+xOffset # edit for grid size
                point.y=i*cells.cell_height+yOffset # edit for grid size
                point.z=0
                wallList.append((j-1,i-1))
                cells.cells.append(point)
            k=k+1
        k=k+1
        if (grid[k] == 100):
            point=Point()
            point.x=j*cells.cell_width+.62 # edit for grid size
            point.y=i*cells.cell_height-.15 # edit for grid size
            point.z=0
            cells.cells.append(point)

        set(wallList)
    #print cells # used for debugging
    print "mapped"
    wallpub.publish(cells)           

def initAstar():
    
    print "entered init astar"    
    
    global ckd
    global front
    global wayPointPub
    global currWay_x
    global currWay_y
    global mapData
    mapData = []
    global mapSub
    global wallpub
    global bud
    global xOdom
    global yOdom
    global xClickPose
    global yClickPose 
    global currWayx
    global currWayy 
    global xOffset
    global yOffset 
    global width
    global height
    global wallList
    global odom_tf
    global odom_list
    xOdom = 0
    yOdom = 0
    xClickPose = 8
    yClickPose = 8
    currWayx = 0
    currWayy = 0
    xOffset = 0
    yOffset = 0
    width = 10
    height = 10
    wallList = []
    ckd = rospy.Publisher("/grid_checked", GridCells, queue_size=1)
    front = rospy.Publisher("/grid_Front", GridCells, queue_size=1)
    wayPointPub = rospy.Publisher('move_base', PoseStamped, queue_size=1)
    mapsub = rospy.Subscriber('/finished', OccupancyGrid, mapCallBack)
    wallpub = rospy.Publisher("/expanded_map", GridCells, latch=True)
    bud = rospy.Publisher('/edge', GridCells, queue_size=1)
    # Subscribe to move base status.
    move_base_status = rospy.Subscriber('/move_base/status', GoalStatusArray, moveBaseStatus)
    
    odom_list = tf.TransformListener()

    rospy.sleep(rospy.Duration(1,0))
    rospy.Timer(rospy.Duration(0.1), readOdom) 
    #pose = Pose()  
        

    # Use this object to get the robot's Odometry 
    #sub = rospy.Subscriber('/odom', Odometry, readOdom)
    

        
    #odom_tf = tf.TransformBroadcaster()
    #odom_tf.sendTransform((0, 0, 0),(0, 0, 0, 1),rospy.Time.now(),"base_footprint","odom")

    #Used to get the pose from the mouse click in RVIZ
    sub = rospy.Subscriber('/move_base_simple/goal/RBE',PoseStamped, readPose)


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
    #pose.position.x = position[0]
    #pose.position.y = position[1]
    xOdom = position[0]
    yOdom = position[1]
    #xPos = int(xPos * 5) + int(width/2)
    #yPos = int(yPos * 5) + int(height/2)
    odomW = orientation
    q = [odomW[0], odomW[1], odomW[2], odomW[3]]
    roll,pitch,yaw = euler_from_quaternion(q)
    theta = yaw#math.degrees(yaw)


#Pose Callback Function.
def readPose(msg):

    #px = msg.pose.position.x
    #py = msg.pose.position.y
    quat = msg.pose.orientation
    q = [quat.x, quat.y, quat.z, quat.w]
    roll, pitch, yaw = euler_from_quaternion(q)
    global newGoalReceived
    global xClickPose
    global yClickPose
    global thetaPose
    global notInitStartup
    newGoalReceived = True
    notInitStartup = True
    xClickPose = msg.pose.position.x
    yClickPose = msg.pose.position.y
    #xEnd = int(px * 5) + int(width/2)
    #yEnd = int(py * 5) + int(height/2)
    thetaPose = yaw   #Determine theta.	


#it's recommended that start is a poseStamped msg and goal is a pose msg, RViz likes using that for visualization.
def AStar(xInit, yInit, xEnd, yEnd, AWidth, AHeight):

    global mapData
    frontier = []
    checked = []
    unchecked = []
    wallBuddies = []

    #print "Awidth: ", AWidth, " AHeight: ", AHeight
    #print "xEnd: ", xEnd, " yEnd: ", yEnd
    #print "x init: ", xInit, "y init", yInit

    # determine if current position is considered to be a wall
    if(getWallVal(xInit, yInit)):
        print "AHHHHHHHHHHHH current location is a wallll: ", getWallVal(xInit, yInit)
            
    #get a list of all neighboring x and y coordiantes 
    neighbors = getNeighbors(xInit, yInit) 

    for i in range(4):

        new_x = neighbors[i][0] 
        new_y = neighbors[i][1] 
        #print "new_x ", new_x, "new_y ",new_y
        #if in bounds (equal to zero less than width)
        #if 0 <= new_x and new_x < width and 0 <= new_y and new_y < height:
        #print "xNeigh: ", new_x, "yNeigh: ", new_y, "wall val: ", getWallVal(new_x, new_y)

    #add a node for every x and y coordiate to the unchecked list
    for x in range(AWidth):
        for y in range(AHeight):
            unchecked.append(GridSquare(x, y, 0, 0, 0, 0, (0,0)))

    #add the wall value for every Grid Square
    for i in range(len(unchecked)):
        currentNode = unchecked[i]
        unchecked[i].wallVal = getWallVal(currentNode.x, currentNode.y)
    
    #add a heuristic value for every node
    for i in range(len(unchecked)):
        currentNode = unchecked[i]
        unchecked[i].h = getHeuristic(xEnd, yEnd, currentNode.x, currentNode.y)

	# starting x and y are the current coordinates
    curr_x = xInit
    curr_y = yInit

    # Determine current index in the unchecked list
    index = getIndexPlace(unchecked, curr_x, curr_y)

    #determines h value
    init_h = unchecked[index].h

    #determines g value 
    init_g = 0

    #determines f value
    init_f = init_g + init_h

    #determines wall value
    init_wallVal = unchecked[index].wallVal
    
    #sets came from value to itself since it is the start node
    init_cameFrom = (curr_x, curr_y)

    #create the first node
    initNode = GridSquare(curr_x, curr_y, init_h, init_g, init_f, init_wallVal, init_cameFrom)   

    # add starting square to frontier
    frontier.append(initNode)

    #remove starting cell from unchecked
    unchecked = nodeRemove(unchecked, curr_x, curr_y)
    
    while(len(frontier)and not rospy.is_shutdown()): # while there are still nodes that have not been checked, continually run the algorithm
        #gets the most likely node from frontier and removes it from frontier
        currentSquare = frontierGetMin(frontier) 
        
       
        #set's current x and y values based on node being expanded
        curr_x = currentSquare.x
        curr_y = currentSquare.y

        #print "currX" , curr_x , "currY" , curr_y
        #print "g" , currentSquare.g, "h" , currentSquare.h , "f" , currentSquare.f
        #print "Yend", yEnd ,"XEnd", xEnd

        #if the current cell is the goal
        if (curr_x == xEnd) and (curr_y == yEnd): 
            print("goal found!") 
            
            #creates the goal node   
            originalCell = GridSquare(curr_x, curr_y, currentSquare.h, currentSquare.g, currentSquare.f, currentSquare.wallVal, currentSquare.cameFrom) 

            #adds the goal node to the checked list 
            checked = nodeAdd(checked, originalCell)
           
            return reconstructPath(checked, xInit, yInit, xEnd, yEnd) 
                                                  
        
        #get a list of all neighboring x and y coordiantes 
        neighbors = getNeighbors(curr_x, curr_y) 
        
        for i in range(4):
            new_x = neighbors[i][0] 
            new_y = neighbors[i][1] 
            
            #if in bounds (equal to zero less than width)
            if 0 <= new_x and new_x < AWidth and 0 <= new_y and new_y < AHeight:
                    
                #if the node is unchecked
                if itemExists(unchecked, new_x, new_y):
                    
                    #sets index value of node in unchecked
                    index = getIndexPlace(unchecked, new_x, new_y)

                    #if the node is not a wall
                    if unchecked[index].wallVal == False:    
                        
				        #set the came from value for the new cell
                        new_cameFrom = (curr_x, curr_y)
		    
				        # set the g value to the g value of the parent node plus one
                        new_g = currentSquare.g + 1
                
                        #get's the h value of the new cell
                        new_h = getHeuristic(xEnd, yEnd, new_x, new_y)

				        #set f value
                        new_f = new_h + new_g

                        #gets old Wall Value
                        new_wallVal = unchecked[index].wallVal

                        #makes new gridCell
                        newCell = GridSquare(new_x, new_y, new_h, new_g, new_f, new_wallVal, new_cameFrom)

                        #add cell to frontier
                        frontier.append(newCell)

                        publishCells(frontier, 2)

                        #remove cell from unchecked
                        unchecked = nodeRemove(unchecked, new_x, new_y)

                    else:
                        #print currentNode.h,
                        currentNode.h += 2
                        #print currentNode.h,
                        wallBuddies.append(currentNode)
                        publishCells(wallBuddies,4) 
                        #print len(wallBuddies)

                elif itemExists(checked, new_x, new_y):

                    #sets index value of node in checked
                    index = getIndexPlace(checked, new_x, new_y)
                    #calculates what the new g value could be
                    tentative_g = currentSquare.g + 1

                    #if the new possible g value is lower than the original
                    if tentative_g < checked[index].g:
                        #update the g value and the came from value
                        checked[index].g = tentative.g
                        checked[index].cameFrom = (curr_x, curr_y)

            

            #creates the current node  
            originalCell = GridSquare(curr_x, curr_y, currentSquare.h, currentSquare.g, currentSquare.f, currentSquare.wallVal, currentSquare.cameFrom) 

            #adds the current cell to the checked list 
            checked = nodeAdd(checked, originalCell)

            # update checked list
            publishCells(checked, 1)

    print "failure" 
    fail = []
    return fail #if the program runs out of nodes to check before it finds the goal, then a solution does not exist

#returns the most likely node on the frontier and removes it from the frontier
def frontierGetMin(frontier):
    lowestF = 100000;
    lowestIVal = 100000;
    for i in range(len(frontier)):
        currentF = frontier[i].f
        if(currentF < lowestF):
            lowestF = currentF
            lowestIVal = i
    nextNode = frontier[lowestIVal]
    frontier.pop(lowestIVal)
    return nextNode
    

# goes through a list and returns the idex for the desired cell
def getIndexPlace(listToSearch, xCoord, yCoord):
    #print len(listToSearc)
    global width
    global height

    #print "width: ", width, " height: ", height
    for i in range(len(listToSearch)):
        listItem = listToSearch[i]
        if( xCoord == listItem.x and yCoord == listItem.y):
            return i
    print "list Item Not Found" 
    print "x: ", xCoord, " y: ", yCoord
    return -10

#checks if a given x and y coordiante are in a given list
def itemExists(listToSearch, xCoord, yCoord):
    for i in range(len(listToSearch)):
        listItem = listToSearch[i]
        if( xCoord == listItem.x and yCoord == listItem.y):
            return True
    return False

#removes given node from given list
def nodeRemove(givenList, xCoord, yCoord):
    #verifies the item is currently in the list
    if itemExists(givenList, xCoord, yCoord):
        i = getIndexPlace(givenList, xCoord, yCoord)
        givenList.pop(i)
        return givenList
    else:
        print "item not in list"
        print "x Val: ", xCoord, " y coord: ", yCoord
        return givenList

#adds given node to given list
def nodeAdd(givenList, givenNode):
    givenList.append(givenNode)
    return givenList

# gets the wall value for the given x and y coordinates and returns the correct value
def getWallVal(xVal, yVal):
    global wallList
    #print wallList
    for set in wallList:
        if(set[0] == xVal and set[1] == yVal):
            return True
    return False
 
#gets the map index value
def getMapIndex(xVal,yVal):   

    global width
    global mapgrid
     
    # use 1 or 0.2 for resolution or 1 or 0.2
    robotResolution = 0.2
    a = (((yVal- mapgrid.info.origin.position.y) / robotResolution) * width)
    a = a + ((xVal - mapgrid.info.origin.position.x) / robotResolution)
    return int(round(a,2))
    

#takes in the x and y value of a node and the end x and y value and returns the correct heurisitic
def getHeuristic(xEnd, yEnd, xGiven, yGiven):
    
    xDist = xEnd - xGiven
    yDist = yEnd - yGiven
    hVal = abs(xDist) + abs(yDist)
    #hVal = math.sqrt(xDist**2 + yDist**2)
    return hVal

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


#returns list of tuples of desired path
def reconstructPath(checked, xInit, yInit, xEnd, yEnd):

    Path = []
    currentCell = (xEnd, yEnd)
    # while the path is not back home yet
    while currentCell[0] != xInit or currentCell[1] != yInit :
        #set x and y values for current cell
        curr_x = currentCell[0]
        curr_y = currentCell[1]
        #print "path x", curr_x , "path y" , curr_y
        # get location of current cell in list
        currentIndex = getIndexPlace(checked, curr_x, curr_y)
        #set the next cell to be the came from cell of the current cell
        nextCell = checked[currentIndex].cameFrom
        # add the current cell to the list
        Path.append(currentCell)
        # change the current cell to be the next cell
        currentCell = nextCell

    initCell = (xInit, yInit)
    Path.append(initCell)
    #print "Path", Path
    return Path

#run's A* to get the path from start to goal, Finds the locations and directions of each waypoint, calls publish gaol to publish the next way point as a poseStamped
def getNextWayPoint():  

    global xOdom
    global yOdom
    global xClickPose
    global yClickPose 
    global currWayx
    global currWayy 
    global xOffset
    global yOffset 
    global width
    global height
    global res

    print 'xoffset', xOffset , 'yoffset', yOffset
    print 'xOdom', xOdom , 'yOdom', yOdom
    print 'xClickPose', xClickPose , 'yClickPose', yClickPose
    print 'height' , height , 'width' , width
    print 'resolution' , res

    AEndGoalx = int((xClickPose - xOffset) / res)
    AEndGoaly = int((yClickPose - yOffset) / res)
    AWayx = int((currWayx - xOffset) / res)
    AWayy = int((currWayy - yOffset) / res)
    ARoPosx = (xOdom - xOffset) / res
    ARoPosy = (yOdom - yOffset) / res
    AWidth = width
    AHeight = height

    AInitx = int(ARoPosx)
    AInity = int(ARoPosy)
 
    print "A* Initial X:", AInitx , "A* Initial Y:" , AInity
    print "A* EndGoalx:", AEndGoalx , "A* EndGoaly:" , AEndGoalx
    print "A* robot Pos x" , ARoPosx, "A* robot Pos y" , ARoPosy

    path = AStar(AInitx, AInity, AEndGoalx, AEndGoaly, AWidth, AHeight)
    AWayPoints = locateWayPointsLocations(path)
    ADirections = locateWayPointsDirections(path)
    lengthWay = len(AWayPoints)
    lengthDir = len(ADirections)
    if (lengthWay < 1 or lengthDir <1):
        print "at the goal"
        return 1
    ANextWay = AWayPoints[lengthWay - 1]
    print "A*wayX" , ANextWay[0] , "A*wayY" , ANextWay[1]
    ANextDir = ADirections[lengthDir - 1]    
    currWayx = float(ANextWay[0] * res) + xOffset
    currWayy = float(ANextWay[1] * res) + xOffset
    print "currWayX" , currWayx , "currWayy" , currWayy
    nextDir = ANextDir
    nextWay = (currWayx , currWayy)
    #publishGoal(ANextWay, ANextDir)
    publishGoal(nextWay, nextDir)
    
#publish next way point as poseStamped
def publishGoal(location, direction):
    
    global wayPointPub
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.pose.position.x = location[0]
    goal.pose.position.y = location[1]
    goal.pose.position.z = 0
    (w, x, y, z) = quaternion_from_euler(0, 0, direction)
    goal.pose.orientation = Quaternion(w, x, y, z)
    wayPointPub.publish(goal)

#locate WayPoints
def locateWayPointsLocations(path):

    listWay =  []

    xDiff = 0
    yDiff = 0

    for i in range(1, len(path)):
        new_xDiff = path[i][0] - path[i-1][0]
        new_yDiff = path[i][1] - path[i-1][1]
        #if the robot changes directions
        if new_xDiff != xDiff or new_yDiff != yDiff:
            #listOfDirections.append(getDirection(xDiff, yDiff))
            listWay.append(path[i - 1])
        xDiff = new_xDiff
        yDiff = new_yDiff

    return listWay

#locate Directions at each wayPoint
def locateWayPointsDirections(path):

    listOfDirections = []

    xDiff = 0
    yDiff = 0

    for i in range(1, len(path)):
        new_xDiff = path[i][0] - path[i-1][0]
        new_yDiff = path[i][1] - path[i-1][1]
        #if the robot changes directions
        if new_xDiff != xDiff or new_yDiff != yDiff:
            listOfDirections.append(getDirection(xDiff, yDiff))
        xDiff = new_xDiff
        yDiff = new_yDiff

    return listOfDirections


#returns the radian value of the angle theta with respect to positive x, if 10 is returned it's an error 
def getDirection(x, y):
    if x == -1:
        if y == 0:
            return 0
        else:
            print "incorrect Direction 1"
            return 10
    if x == 0:
        if y == -1:
            return math.pi / 2
        elif y == 1: 
            return - math.pi / 2
        elif y ==0: 
            return 0
        else:
            print "incorrect Direction 2"
            return 10
    elif x == 1:
        if y == 0:
            return math.pi
        else:
            print "incorrect Direction 3"
            return 10
    else: 
        print "incorrect Direction 4"
        return 10

#grid is the list of points that have been checked
#num is designating whether it is a 2-front, 1-checked, or 100-wall
def publishCells(grid,num):
    global ckd
    global bud
    global front
    global xOffset
    global yOffset
    #print "publishing"
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
    if(num == 100):
        pub.publish(cells)           
    if(num == 1):
        ckd.publish(cells)
    if(num == 2):
        front.publish(cells)
    if(num == 4):
        bud.publish(cells)


'''def runAstar():

    print "starting run astar"
    # initialize astar publishers and subscribers
    initAstar()
    
    while 1 and not rospy.is_shutdown():
        rospy.sleep(1)
        #print "start pose"
        #print xInit
        #print yInit
        #print "end pose"
        #print xEnd
        #print yEnd
        getNextWayPoint()
        #print "goal reached!!!"
        #print xEnd
        #print yEnd
        #print("complete")
        #rospy.loginfo("Complete")
        #rospy.spin() '''


if __name__ == '__main__':

    rospy.init_node('bshappell_kcorton_nkjefferson_finalA')
    
    global xOdom
    global yOdom
    global xClickPose
    global yClickPose 
    global newGoalReceived

    global goalReached
    global goalUnreachable
    global notInitStartup

    newGoalReceived = False
    goalReached = False
    goalUnreachable = False
    notInitStartup = False

    initAstar()
    


    while 1 and not rospy.is_shutdown():
        rospy.sleep(1)
        if((xClickPose - .5) < xOdom < (xClickPose + .5)):
            if((yClickPose - .5) < yOdom < (yClickPose + .5)):
                robotAtFinalGoal = True 
        else:
            robotAtFinalGoal = False

        print "robotAtFinalGoal" , robotAtFinalGoal
        print "goalReached" , goalReached

        while(not robotAtFinalGoal and not rospy.is_shutdown()):
            if(newGoalReceived):
                newGoalReceived = False
                getNextWayPoint() 
            if(goalReached):
               getNextWayPoint()
            if(goalUnreachable):
                print "goal unreachable in A*" 

