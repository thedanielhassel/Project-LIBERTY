#! /usr/bin/env python

#script containing a database of the major directions used 

#import the math librarie
import math

#make the calss for checking directions
class check_direction:

    #initialize the values
    def __init__(self):
        self.N = 0.0
        self.S = 1.0 #could be -1 or 1 (0 deg = 1, 360 deg = -1)
        self.E = -math.sqrt(2)/2
        self.W = math.sqrt(2)/2
        self.NE = -math.sqrt(2 - math.sqrt(2))/2
        self.NW = math.sqrt(2 - math.sqrt(2))/2
        self.SE = -math.sqrt(math.sqrt(2) + 2)/2
        self.SW = math.sqrt(math.sqrt(2) + 2)/2


    #function for returning the value corresponding with a major direction
    def getDir(self, dir):
        if dir == 'N':
            return self.N
        if dir == 'S':
            return self.S
        if dir == 'E':
            return self.E
        if dir == 'W':
            return self.W
        if dir == 'NE':
            return self.NE
        if dir == 'NW':
            return self.NW
        if dir == 'SE':
            return self.SE
        if dir == 'SW':
            return self.SW
        else:
            return 'False'