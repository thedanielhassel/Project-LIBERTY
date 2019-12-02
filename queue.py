#! /usr/bin/env python

#scripts that functions as a wrapper for the heapq library

#import the heapq library
import heapq

#define a class for using it
class priorityQueue:

    #initialization
    def __init__(self):
        self.elements = []

    #function to empty the queue
    def empty(self):
        return len(self.elements) == 0

    #function to put an item in the queue
    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))

    #function to extract next item from the queue
    def get(self):
        return heapq.heappop(self.elements)[1]