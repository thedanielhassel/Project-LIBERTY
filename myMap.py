#! /usr/bin/env python

#script containing the class that handles the map generated from hector_slam

#import the position.py script
import position as posModule

#create class
class myMap:

    #initialization 
    def __init__(self, theMap, height, width, res):  
        self.mapData = theMap
        self.height = height
        self.width = width
        self.resolution = res
    
    #fucntion for returning the map data
    def get_mapData(self):
        return self.mapData 
    
    #function setting a specified node value 
    def set_mapData(self, node, val):
        self.mapData[node] = val

    #function returning the map height
    def get_height(self): 
        return self.height

    #function returning the map width
    def get_width(self): 
        return self.width

    #function returning the map resolution
    def get_resolution(self): 
        return self.resolution

    #function returning the number of nodes in the map
    def get_number_of_nodes(self): 
        return (self.height * self.width)

    #function returning the map size in square meters
    def get_map_size(self): 
        return ((self.height * self.resolution) * (self.width * self.resolution))

    #function that translates a specified node into its position in the map (meters)
    def node_to_pos(self, node):
        x_pos = ((((node % self.width)) - (self.width / 2)) * (self.resolution) + (self.resolution / 2))
        y_pos = (((((node - (node % self.width)) / self.height) - (self.height / 2)) * self.resolution) + (self.resolution / 2))
        return posModule.position(x_pos, y_pos)

    #function that translates a specified node into its gridposition in the map (nodes)
    def node_to_gridpos(self, node):  
        x_pos = (node % self.width)
        y_pos = ((node - x_pos) / self.height)     
        return posModule.position(x_pos, y_pos)
    
    #function that translates a given position (meters) to its corresponding node
    def pos_to_node(self, pos):
        x = (int(((pos.getX() - (self.resolution / 2)) / self.resolution)) + int((self.width / 2)))
        y = (int((((pos.getY() - (self.resolution / 2)) / self.resolution) + (self.height / 2))) * int(self.height))
        return (x + y)

    #function that translates a given position (meters) to its corresponding position (nodes)
    def pos_to_gridpos(self, pos):
        return self.node_to_gridpos(self.pos_to_node(pos))

    #funcrtion that checksif a node is withing the boundaries of the map
    def inMap(self, node):
        temp = self.node_to_gridpos(node)
        x = temp.getX()
        y = temp.getY()
        if x != 0 and x != self.width - 1:
            return ((0 <= x < self.width) and (0 <= y < self.height))
        else:
            return False

    #function that checks if a node is occupied
    def noWall(self, node):
        return self.mapData[node] != 100
    
    #function that checks if all nodes surrounding a specified node are not occupied
    def space(self,node):
        if node >= (self.width + 1):
            a1 = self.mapData[node - (self.width + 1)] != 100
        else:
            a1 = False
        if node >= (self.width):     
            a2 = self.mapData[node - (self.width)] != 100
        else:
            a2 = False
        if node >= (self.width - 1): 
            a3 = self.mapData[node - (self.width -1)] != 100 
        else:
            a3 = False
        if node >= 1:
            a4 = self.mapData[node - 1] != 100
        else:
            a4 = False
        if node < ((self.height * self.width) - 1): 
            a5 = self.mapData[node + 1] != 100
        else:
            a5 = False
        if node < ((self.height * self.width) - (self.width - 1)): 
            a6 = self.mapData[node + (self.width - 1)] != 100
        else:
            a6 = False 
        if node < ((self.height * self.width) - (self.width)): 
            a7 = self.mapData[node + (self.width)] != 100
        else:
            a7 = False
        if node < ((self.height * self.width) - (self.width + 1)):  
            a8 = self.mapData[node + (self.width + 1)] != 100
        else:
            a8 = False
        if (a1 == True) and (a2 == True) and (a3 == True) and (a4 == True) and 
           (a5 == True) and (a6 == True) and (a7 == True ) and (a8 == True):
            return True
        else:
            return False

    #function that filters and returns the available neighbours of a node
    def nextTo(self, node):

        #specifies what a neighbour is
        n = [node - (self.width + 1), node - (self.width), node - (self.width - 1),
             node - 1, node + 1, node + (self.width - 1), node + (self.width), 
             node + (self.width + 1)]
        
        #uses filter functions to filter out "bad" nodes
        n = filter(self.inMap, n)
        n = filter(self.noWall, n)
        n = filter(self.space, n)
        return n
    
    #function defining which direction the rover should take between two nodes
    def next_move(self, current_node, next_node):
        move = current_node - next_node
        if move == 1:
            return 'W'
        if move == - 1:
            return 'E'
        if move == self.width:
            return 'S'
        if move == -self.width:
            return 'N'
        if move == self.width + 1:
            return 'SW'
        if move == self.width - 1:
            return 'SE'
        if move == -(self.width + 1):
            return 'NE'
        if move == -(self.width - 1):
            return 'NW'
        