#! /usr/bin/env python

#script containing the implementation of A*

#importing the necessary libraries
import math

#importing the scripts needed
import myMap as mapModule
import queue as queueModule
import directionDatabase as ddModule

#function used to make a path from start to finish
def makePath(cameFrom, start, finish):

    #set the finish and setup path list
    current = finish
    path = []

    #while we are not at the start
    while current != start:

        #put the curent node in the path
        path.append(current)

        #retrive its parent and set it as the current node
        current = cameFrom[current]

    #place the start node in the path
    path.append(start)

    #reverses it so that it can be used from the rovers perspective
    path.reverse() 
    return path

#function used to set the starting nodes parents
def setCameFromStart(dirr, start, theMap):

    #generate direction database object and calculate the rovers current direction
    move_dir = ddModule.check_direction()
    d = math.acos(dirr) * 2
    step = math.pi / 8
    currentPlus = d + step
    currentMinus = d - step
    temp = 0

    #find the major direction that corresponds to the rovers back 
    #set said direction as its parent
    if currentMinus < math.acos(move_dir.getDir('N')) * 2 < currentPlus:
        temp = start + theMap.get_width()
        print('Facing: N')
    if currentMinus < math.acos(move_dir.getDir('NE')) * 2 < currentPlus:
        temp = start + (theMap.get_width() - 1)
        print('Facing: NE')
    if currentMinus < math.acos(move_dir.getDir('E')) * 2 < currentPlus:
        temp = start - 1
        print('Facing: E')
    if currentMinus < math.acos(move_dir.getDir('SE')) * 2 < currentPlus:
        temp = start - (theMap.get_width() + 1)
        print('Facing: SE')
    if (currentMinus < math.acos(move_dir.getDir('S')) * 2 < currentPlus) or (0 < math.acos(move_dir.getDir('S')) * 2 < step):
        temp = start - theMap.get_width()
        print('Facing: S')
    if currentMinus < math.acos(move_dir.getDir('SW')) * 2 < currentPlus:
        temp = start - (theMap.get_width() - 1)
        print('Facing: SW')
    if currentMinus < math.acos(move_dir.getDir('W')) * 2 < currentPlus:
        temp = start + 1
        print('Facing: W')
    if currentMinus < math.acos(move_dir.getDir('NW')) * 2 < currentPlus:
        temp = start + (theMap.get_width() + 1)
        print('Facing: NW')
    return temp
        
#function used to solve the actual pathfinding
def aStarSolver(theMap, start, finish, currentDir):

    #set the goal and add the start to the open list 
    posFinish = theMap.node_to_gridpos(finish)
    openList.put(start, 0)

    #generate lists for parents and costs
    cameFrom = {}
    currentCost = {}

    #set parents to none and cost to inf for the beginning
    for i in range(0, int(theMap.get_number_of_nodes() - 1)):
        cameFrom[i] = None
        currentCost[i] = float("inf")

    #set the cost of starting to 0
    currentCost[start] = 0

    #get the starts parent using setCameFromStart fucntion
    cameFrom[start] = setCameFromStart(currentDir, start, theMap)

    pathfound = False

    #while there are objects in the open list
    while not openList.empty():

        #get the object with the lowest cost
        current = openList.get()

        #check if it is the goal
        if current == finish:
            print("Found path")

            #break the loop if it is, the goal has been reached
            pathfound = True
            break
        
        #check all the current nodes neighbours for their cost and availability
        for next in theMap.nextTo(current):

            #if the node is to the currents NW, NE, SW or SE
            if (next == current + theMap.get_width() + 1) or (next == current + theMap.get_width() - 1) or (next == current - theMap.get_width() + 1) or (next == current - theMap.get_width() - 1 ):
                
                #if node infront of rover
                if (current - cameFrom[current]) == (next - current):

                    #set the cost to current + 2
                    newCost = currentCost[current] + 2

                #if it is not infront
                else:

                    #set the cost to current + 3
                    newCost = currentCost[current] + 3
                
            #if the node is to the currents N, S, E og W
            else:

                #if the node is infront of the rover
                if (current - cameFrom[current]) == (next - current):

                    #set the cost to current  + 1
                    newCost = currentCost[current] + 1

                #if it is not infront
                else:

                    #set the cost to current + 3
                    newCost = currentCost[current] + 3

            #if the next node is not the currentCost list of the newCost is smaller
            #than the pervious cost
            if next not in currentCost or newCost < currentCost[next]:

                #we ser the new cost of the node
                currentCost[next] = newCost

                #calculates its priority
                priority = newCost + posFinish.dist(theMap.node_to_gridpos(current))

                #places it in the openList
                openList.put(next, priority)

                #selects next node 
                cameFrom[next] = current
        
    #if the path is found
    if pathfound == True:

        #we make the path using the makePath() function
        path = makePath(cameFrom, start, finish)
        path_drawn = []

        #we draw the path in node form
        for next in path:
            temp = theMap.node_to_pos(next)
            path_drawn.append(temp)
        
        #finally we return the path
        return path, path_drawn
    
    #there is no path to the goal
    else:

        #we return the none-values for the path
        return 0, 0