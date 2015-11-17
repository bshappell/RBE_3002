#!/usr/bin/env python

import math 
import numpy
class GridSquare:
    def __init__(self, checked, h, g, cameFrom):
        self.checked = checked # 0 = unchecked, 1 = checked, 2 = wall,  3 = frontier
        self.h = h
        self.g = g
        self.cameFrom = cameFrom 

class FrontierSquare:
    def __init__(self, x, y, f):
        self.x = x
        self.y = y
        self.f = f

#it's recommended that start is a poseStamped msg and goal is a pose msg, RViz likes using that for visualization.
def AStar(ixInit, iyInit, ixEnd, iyEnd, iwidth, iheight):

    global width
    global height
    global xEnd
    global yEnd
    global xInit
    global yInit
    global mapInfo

    xInit = ixInit
    yInit = iyInit
    xEnd = ixEnd
    yEnd = iyEnd
    width = iwidth
    height = iheight


    frontier = [] # list of frontier squares 

    mapInfo = [] # 2d array of grid square info
    row = []
    for i in range(height):
        row.append(GridSquare(0, 0, 0, (0,0))) # append empty grid cell
    for i in range(width):
        mapInfo.append(row)
    print "width", len(mapInfo)
    print "height", len(mapInfo[0])

    #print mapInfo[0]


    # calculate out heursitics at each coordinate
    #getHeuristic()

	# starting x and y are the current coordinates
    curr_x = xInit
    curr_y = yInit

    # Determine current f value
    print "h and g"
    print mapInfo[curr_x][curr_y].h,  mapInfo[curr_x][curr_y].g
    curr_f = mapInfo[curr_x][curr_y].h + mapInfo[curr_x][curr_y].g

    # add starting square to frontier
    frontier.append(FrontierSquare(curr_x, curr_y, curr_f))
    print "frontier info"
    print frontier[0].x
    print frontier[0].y
    print frontier[0].f

    while(len(frontier)): # while there are still nodes that have not been checked, continually run the algorithm
	
        currentSquare = frontier[0] # this is the most promising node of all nodes in the open set
        print "cur_x ", curr_x, " curr_y ", curr_y
        frontier.remove(currentSquare) # remove currentSquare from the frontier
        
        curr_x = currentSquare.x
        curr_y = currentSquare.y

        if (curr_x == xEnd) and (curr_y == yEnd): # if the best possible path found leads to the goal, it is the best possible path that the robot could discover
            print("goal reached")            
            return reconstructPath()
         
        neighbors = getNeighbors(currentSquare) # re-evaluate each neighboring node
        
        # add from frontier
        for neighbor in neighbors:
            frontier.append(neighbor)
            print neighbor.x
            print neighbor.y
            print neighbor.f
        # sort frontierList by f (g(s) + h(s))        
        frontier.sort(key=lambda cell: cell.f)
    print "failure" 
    return -1 #if the program runs out of nodes to check before it finds the goal, then a solution does not exist


#mapInfo, is list of list of GridSquares
def getHeuristic():
    global xEnd
    global yEnd
    global width
    global height
    global mapInfo

    for x in range(width):
	for y in range(height):
	    xDist = xEnd - x
	    yDist = yEnd - y
            print "xDist ", xDist, " yDist ", yDist, " x ", x, " y ", y
	    #print math.sqrt((xDist**2) + (yDist**2))
	    print mapInfo[x][y].h
	    mapInfo[x][y].h = math.sqrt((xDist**2) + (yDist**2))

    #print "map h", mapInfo.h
    #print (math.sqrt(2**2 + 4**2))


#Sets x,y to checked
#Sets came from in checked neighbors
#Checks if wall and sets checked status true on walls
#Returns list of cardinal neighbors tuple
def getNeighbors(currentCell):
	global width
	global height
        global mapInfo
	#print "width"
        #print width
        #print "height"
        #print height
	#set current cell to checked
	mapInfo[currentCell.x][currentCell.y].checked = 1
    #check neighbor to north
	delta_x = [0,1,0,-1]
	delta_y = [1,0,-1,0]
	frontierList = []

	for i in range(4):
		new_x = currentCell.x + delta_x[i]
		new_y = currentCell.y + delta_y[i]
                print "new_x ", new_x, "new_y ",new_y
		#if in bounds (equal to zero less than width)
		if 0 <= new_x and new_x < width and 0 <= new_y and new_y < height:
                    #print "new_x ", new_x, " new_y ",new_y
                    #print "cell x ", currentCell.x, " cell y ", currentCell.y
			#if the neighbor cell has not been checked
		    if mapInfo[new_x][new_y].checked == 0:
				#set the came from value for the new cell
	    	        mapInfo[new_x][new_y].cameFrom = (currentCell.x, currentCell.y)
				#set the cell checked value as frontier
			mapInfo[new_x][new_y].checked = 3
				# set the g value to the g value of the parent node plus one
			mapInfo[new_x][new_y].g =mapInfo[currentCell.x][currentCell.y].g + 1 
				#append the new cell to the frontierList
			frontierList.append(FrontierSquare(new_x, new_y, mapInfo[new_x][new_y].g+mapInfo[new_x][new_y].h))
                        print mapInfo[new_x][new_y]
                    elif((mapInfo[new_x][new_y].checked == 1) or (mapInfo[new_x][new_y].checked == 3)):
                        tentative_g = mapInfo[currentCell.x][currentCell.y].g + 1
                        if tentative_g < mapInfo[new_x][new_y].g:
	                    mapInfo[new_x][new_y].g = tentative_g
	                    mapInfo[new_x][new_y].cameFrom = (currentCell.x, currentCell.y)

	#return a list FrontierSquares
	return frontierList

#returns the distance from the initial pose to the current position
#looks at where the neighbor came from and then adds one to that g value


	return 1

#returns array of PoseStamps
def reconstructPath():
	global xInit
	global yInit
	global yEnd
	global xEnd
        global mapInfo

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
def makeList():
	global height
	global width
        global mapInfo

	gridList = []

	
	for y in range(height):
            for x in range(width):
		girdList.append(mapInfo[x][y].checked)


	return gridList


#make a list to print the final path 
def makePathList(Path):
	global height
	global width

	pathList = []

	
	for y in range(height):
            for x in range(width):
		for i in range(len(Path)):
		    if( y == Path[i][1] and x == Path[i][0]):
			pathList.append(50)
		    else:
			pathList.append(0)

	return pathList



