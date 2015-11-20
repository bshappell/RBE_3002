#!/usr/bin/env python

#line 28, 29 and 30 is still throwing an error somehow.... it says line 31 is incorrect but if you comment out 28,29 and 30 it works just fine
#line 256 is turning a list of locations and corresponding dirrections to poseStamped messages.... Theres a not there too but i didn't figure out how to do that, but it should be fairly straight forward hopefully
#line 183, i don't know how we get wall data, but this should return a value, 0 or 100 for a given x and y coordinate based on if it's a wall or not

import math 
import numpy
import Queue
class GridSquare:
    def __init__(self, x, y, h, g, f, wallVal, cameFrom):
        self.x = x
        self.y = y
        self.h = h
        self.g = g
        self.f = f
        self.wallVal = wallVal
        self.cameFrom = cameFrom 


#it's recommended that start is a poseStamped msg and goal is a pose msg, RViz likes using that for visualization.
def AStar(xInit, yInit, xEnd, yEnd, width, height):

    frontier = []
    checked = []
    unchecked = []

    #add a node for every x and y coordiate to the unchecked list
    for x in range(width):
        for y in range(height):
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
    unchecked = nodeRemove(unchecked, xInit, yInit)
    

    while(len(frontier)): # while there are still nodes that have not been checked, continually run the algorithm
	    
        #gets the most likely node from frontier and removes it from frontier
        currentSquare = frontierGetMin(frontier) 
       
        #set's current x and y values based on node being expanded
        curr_x = currentSquare.x
        curr_y = currentSquare.y

        print "currX" , curr_x , "currY" , curr_y
        print "g" , currentSquare.g, "h" , currentSquare.h , "f" , currentSquare.f

        #if the current cell is the goal
        if (curr_x == xEnd) and (curr_y == yEnd): 
            print("goal reached!") 
            
            #creates the current node  
            originalCell = GridSquare(curr_x, curr_y, currentSquare.h, currentSquare.g, currentSquare.f, currentSquare.wallVal, currentSquare.cameFrom) 

            #adds the current cell to the checked list 
            checked = nodeAdd(checked, originalCell)
           
            reconstructPath(checked, xInit, yInit, xEnd, yEnd) 
            return 1                                       
        
        #get a list of all neighboring x and y coordiantes 
        neighbors = getNeighbors(curr_x, curr_y) 
        
        for i in range(4):
            new_x = neighbors[i][0] 
            new_y = neighbors[i][1] 
            #print "new_x ", new_x, "new_y ",new_y
            #if in bounds (equal to zero less than width)
            if 0 <= new_x and new_x < width and 0 <= new_y and new_y < height:
                
                #if the node is unchecked
                if itemExists(unchecked, new_x, new_y):
                    
                    #sets index value of node in unchecked
                    index = getIndexPlace(unchecked, new_x, new_y)

                    #if the node is not a wall
                    if unchecked[index].wallVal == 0:    
                        
				        #set the came from value for the new cell
                        new_cameFrom = (curr_x, curr_y)
		    
				        # set the g value to the g value of the parent node plus one
                        new_g = currentSquare.g + 1
                
                        #get's the h value of the new cell
                        new_h = unchecked[index].h

				        #set f value
                        new_f = new_h + new_g

                        #gets old Wall Value
                        new_wallVal = unchecked[index].wallVal

                        #makes new gridCell
                        newCell = GridSquare(new_x, new_y, new_h, new_g, new_f, new_wallVal, new_cameFrom)

                        #add cell to frontier
                        frontier.append(newCell)

                        #remove cell from unchecked
                        unchecked = nodeRemove(unchecked, new_x, new_y)
                        

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

    print "failure" 
    return -1 #if the program runs out of nodes to check before it finds the goal, then a solution does not exist

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
    for i in range(len(listToSearch)):
        listItem = listToSearch[i]
        if( xCoord == listItem.x and yCoord == listItem.y):
            return i
    print "list Item Not Found" 
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
        return givenList

#adds given node to given list
def nodeAdd(givenList, givenNode):
    givenList.append(givenNode)
    return givenList

# gets the wall value for the given x and y coordinates and returns the correct value
def getWallVal(xVal, yVal):
    #change this later!
    return 0


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

#returns array of PoseStamps
def reconstructPath(checked, xInit, yInit, xEnd, yEnd):

    Path = []
    currentCell = (xEnd, yEnd)
    # while the path is not back home yet
    while currentCell[0] != xInit or currentCell[1] != yInit:
        #set x and y values for current cell
        curr_x = currentCell[0]
        curr_y = currentCell[1]
        print "path x", curr_x , "path y" , curr_y
        # get location of current cell in list
        currentIndex = getIndexPlace(checked, curr_x, curr_y)
        #set the next cell to be the came from cell of the current cell
        nextCell = checked[currentIndex].cameFrom
        # add the current cell to the list
        Path.append(currentCell)
        # change the current cell to be the next cell
        currentCell = nextCell

    #if we have reached the end
    #locateWayPoints(Path)
    

#locate WayPoints
def locateWayPoints(path):

    listWay =  []
    listOfDirections = []
    listOfPoses = []

    xDiff = 0
    yDiff = 0

    for i in range(1, len(path)):
        new_xDiff = path[i].x - path[i-1].x
        new_yDiff = path[i].y - path[i-1].y
        #if the robot changes directions
        if new_xDiff != xDiff or new_yDiff != yDiff:
            listOfDirections.append(getDirection(xDiff, yDiff))
            listWay.append(path[i - 1])
        xDiff = new_xDiff
        yDiff = new_yDiff

    #turn data into list of poses
    for i in range(0, len(listWay)):
        newX = listPoses[i][0]
        newY = listPoses[i][1]
        newDirection = listOfDirections[i]
        print "x" , newX, "y" , newY , "direction" , newDirection

	return listOfPoses

#returns the radian value of the angle theta with respect to positive x, if 10 is returned it's an error 
def getDirection(x, y):
    if x == -1:
        if y == 0:
            return 0
        else:
            print "incorrect Direction"
            return 10
    if x == 0:
        if y == -1:
            return math.pi / 2
        elif y ==1: 
            return - math.pi / 2
        else:
            print "incorrect Direction"
            return 10
    elif x == 1:
        if y == 0:
            return math.pi
        else:
            print "incorrect Direction"
            return 10
    else: 
        print "incorrect Direction"
        return 10
	
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



