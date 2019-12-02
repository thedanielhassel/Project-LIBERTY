#! /usr/bin/env python

#script containing our position class

#generates the class
class position:

    #initializes it
    def __init__(self, x, y):
        self.x = x 
        self.y = y
    
    #function for setting X value
    def setX(self, x):
        self.x = x

    #function for getting X value
    def getX(self):
        return self.x

    #function for setting Y value
    def setY(self, y):
        self.y = y

    #function for getting Y value
    def getY(self):
        return self.y

    #function for getting distance between two points on the map
    def dist(self, pos):
        return abs(self.x - pos.getX()) + abs(self.y - pos.getY())
    