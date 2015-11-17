#!/usr/bin/env python

import math 
import numpy
import Queue
class GridSquare:
    def __init__(self, h, g, cameFrom):
        #self.check = 0 # 0 = unchecked, 1 = checked, 2 = wall,  3 = frontier
        self.h = h
        self.g = g
        self.cameFrom = cameFrom 


#it's recommended that start is a poseStamped msg and goal is a pose msg, RViz likes using that for visualization.
def AStar(xInit, yInit, xEnd, yEnd, width, height):

    frontier = Queue.PriorityQueue()

    mapInfo = [] # 2d array of grid square info
    row = []
    for i in range(width):
        row.append(GridSquare(0, 0, (0,0))) # append empty grid cell
    for i in range(height):
        mapInfo.append(row)
    print "width", len(mapInfo)
    print "height", len(mapInfo[0])

    checked = []
    checkedRow = []
    for i in range(width):
        checkedRow.append(0) # append empty grid cell
    for i in range(height):
        checked.append(checkedRow)

    #print mapInfo[0]


    # calculate out heursitics at each coordinate
    mapInfo = getHeuristic(mapInfo, xEnd, yEnd, width, height)

	# starting x and y are the current coordinates
    curr_x = xInit
    curr_y = yInit

    # Determine current f value
    print "h and g"
    print mapInfo[curr_x][curr_y].h,  mapInfo[curr_x][curr_y].g
    curr_f = mapInfo[curr_x][curr_y].h + mapInfo[curr_x][curr_y].g

    # add starting square to frontier
    frontier.put((curr_f, (curr_x, curr_y)))
    '''print "frontier info"
    print frontier[0].x
    print frontier[0].y
    print frontier[0].f'''

    while(not frontier.empty()): # while there are still nodes that have not been checked, continually run the algorithm
	
        currentSquare = frontier.get() # this is the most promising node of all nodes in the open set
        #print "cur_x ", curr_x, " curr_y ", curr_y, "ends", xEnd,yEnd
        #frontier.remove(currentSquare) # remove currentSquare from the frontier
        _ , point = currentSquare
        curr_x, curr_y = point
        print "curr_x: ", curr_x, "curr_y: ", curr_y

        #mapInfo[curr_x][curr_y].checked = 1
        if(curr_x != 4):
            print "AAHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH"

        if (curr_x == xEnd) and (curr_y == yEnd): # if the best possible path found leads to the goal, it is the best possible path that the robot could discover
            print("goal reached")            
            return reconstructPath(mapInfo, xInit, yInit, xEnd, yEnd)
         
        neighbors = getNeighbors(curr_x, curr_y) # re-evaluate each neighboring node
        #print "checked val1: ", mapInfo[curr_x][curr_y].checked
        # add from frontier
        for neighbor in neighbors:
            new_x, new_y = neighbor 
            #if in bounds (equal to zero less than width)
	    if 0 <= new_x and new_x < width and 0 <= new_y and new_y < height:
                print "astar_new_x ", new_x, "astar_new_y ",new_y
                #print "cell x ", currentCell.x, " cell y ", currentCell.y
		#if the neighbor cell has not been checked
	        print "checked val: ", checked[new_x][new_y]
		if checked[new_x][new_y] == 0:
				    #set the came from value for the new cell
	    	    mapInfo[new_x][new_y].cameFrom = (curr_x, curr_y)
				    #set the cell checked value as frontier
		    updateChecked(new_x, new_y, checked, 3) #mapInfo[new_x][new_y].check = 3
				    # set the g value to the g value of the parent node plus one
		    mapInfo[new_x][new_y].g =mapInfo[curr_x][curr_y].g + 1 
				#append the new cell to the frontierList
               	    print "added x: ", new_x, "added_y: ", new_y
		    frontier.put((mapInfo[new_x][new_y].g+mapInfo[new_x][new_y].h, (new_x, new_y)))
                        #print mapInfo[new_x][new_y]
                elif((checked[new_x][new_y] == 1) or (checked[new_x][new_y] == 3)):
                    tentative_g = mapInfo[curr_x][curr_y].g + 1
                    if tentative_g < mapInfo[new_x][new_y].g:
	                mapInfo[new_x][new_y].g = tentative_g
	                mapInfo[new_x][new_y].cameFrom = (curr_x, curr_y)
        updateChecked(curr_x, curr_y, checked, 1) #mapInfo[curr_x][curr_y].check = 1

    # print values of checked
    for i in range(10):
        for z in range(10):
            print "x: ", z, "y: ", i, "checked: ", checked[z][i]
    print "failure" 
    return -1 #if the program runs out of nodes to check before it finds the goal, then a solution does not exist

def updateChecked(x_val, y_val, checked, checkedVal):
    checked[x_val][y_val] = checkedVal
    if((x_val != 4) and (checkedVal == 1)):
        print "ERRRRRRRRRRRRRR MYYYYYYYYYYYYYYYYYY GERRRRRRRRRRRRRRDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD"

#mapInfo, is list of list of GridSquares
def getHeuristic(mapInfo, xEnd, yEnd, width, height):
    
    for x in range(width):
	for y in range(height):
	    xDist = xEnd - x
	    yDist = yEnd - y
            #print "xDist ", xDist, " yDist ", yDist, " x ", x, " y ", y
	    #print math.sqrt((xDist**2) + (yDist**2))
	    #print mapInfo[x][y].h
	    mapInfo[x][y].h = abs(xDist) + abs(yDist)#math.sqrt((xDist**2) + (yDist**2))
	    print mapInfo[x][y].h

    #print "map h", mapInfo.h
    #print (math.sqrt(2**2 + 4**2))

    return mapInfo

#Sets x,y to checked
#Sets came from in checked neighbors
#Checks if wall and sets checked status true on walls
#Returns list of cardinal neighbors tuple
def getNeighbors(curr_x, curr_y):

    neighbors = []
	
    #check neighbor to north
    delta_x = [0,1,0,-1]
    delta_y = [1,0,-1,0]
    print "neighbors x: ", curr_x, "neighbors y: ", curr_y
    for i in range(4):
        new_x = curr_x + delta_x[i]
	new_y = curr_y + delta_y[i]
        point = (new_x,new_y)
	neighbors.append(point)
        print "new_x ", new_x, "new_y ",new_y
		
    return neighbors

#returns the distance from the initial pose to the current position
#looks at where the neighbor came from and then adds one to that g value

#returns array of PoseStamps
def reconstructPath(mapInfo, xInit, yInit, xEnd, yEnd):

	Path = []
	currentCell = (xEnd, yEnd)

	while currentCell[0] != xInit or currentCell[1] != yInit:
		nextCell = mapInfo[currentCell[0]][currentCell[1]].cameFrom
		Path.append(currentCell)
		currentCell = nextCell

    #if we have reached the end
	return Path

#locate WayPoints
def locateWayPoints(Path):
	
	lengthOfList = len(Path)

	listWayPoints =  []

	for i in range(0, lengthOfList):
		xFirst = Path[i][0]
		yFirst = Path[i][1]
		
		xThird = Path[i + 2][0]
		yThird = Path[i + 2][1]

		if sqrt((yThird-yFirst)**2 + (yThird - yFirst)**2) > 2:
			listWayPoints.append(Path[i +1])

	return listWayPoints
	
#make a list of A star grids to be printed
def makeList(height,width):

	gridList = []

	
	for y in range(height):
            for x in range(width):
		gridList.append(mapInfo[x][y].check)


	return gridList


#make a list to print the final path 
def makePathList(Path, height, width):

	pathList = []

	
	for y in range(height):
            for x in range(width):
		for i in range(len(Path)):
		    if( y == Path[i][1] and x == Path[i][0]):
			pathList.append(50)
		    else:
			pathList.append(0)

	return pathList



