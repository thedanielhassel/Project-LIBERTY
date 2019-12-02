#! /usr/bin/env python

#script used for running the brain for our autonomous system

#importing the necessary libraries
import rospy
import message_filters
import time
import math

#importing necessary message types
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped

#importing neccesary scripts
import position as posModule
import myMap as mapModule
import aStar as aStarModule
import drawLine as drawLineModule
import action as actionModule
import sendMessage as sendMessageModule

#callback function used for running the enitre process 
def callback(map_data, pos_data):

    #ser the start time 
    start_time = time.time()

    #place map and position data into classes created in myMap.py and position.py
    theMap = mapModule.myMap(map_data.data, map_data.info.height, map_data.info.width, map_data.info.resolution)
    myPos = posModule.position(pos_data.pose.position.x, pos_data.pose.position.y)

    #calculate the node of the current position
    myPosNode = theMap.pos_to_node(myPos)

    #send current position via UDP to specified IP, PORT
    sendCurrentPos = str(myPos.getX()) + ',' + str(myPos.getY())
    sendMessageModule.sendinstructions('192.168.84.254', 2089, sendCurrentPos)
    
    #set the goal of the operation in meters and in node
    myGoal = posModule.position(-0.25, 0.5) 
    myGoalNode = theMap.pos_to_node(myGoal)

    #get current direction
    myDirection = pos_data.pose.orientation.z

    #get the path from the aStarSolver from aStar.py
    path, path_drawn = aStarModule.aStarSolver(theMap, myPosNode, myGoalNode, myDirection)
    
    #draw the RVIZ line from drawLine.py
    drawLineModule.line_draw(path_drawn, myPos)

    #check if there is a path to the goal and set the direction of next move
    if (path != 0) and (myPosNode != myGoalNode):
        direction = theMap.next_move(path[0], path[1])
    else:
        direction = False

    #determine the action needed to get to the next position and send the instructions
    g = actionModule.determine_action(path, direction, myDirection, myGoalNode, myPosNode)
   
    #create publishers for later analysis using rosbags
    #publishes current and wanted directions to two different topics
    pub = rospy.Publisher('goal_out_pose', PoseStamped, queue_size=3)
    goalPose = PoseStamped()
    goalPose.pose.orientation.z = g
    pub.publish(goalPose)
    pub2 = pub = rospy.Publisher('current_out_pose', PoseStamped, queue_size=3)
    currentPose = PoseStamped()
    c = pos_data.pose.orientation.z
    currentPose.pose.orientation.z = math.acos(c) * 2
    pub2.publish(currentPose)
    
    #calculates and prints the elapsed time in order to monitor the time usage of the prosess
    print('elapsed time: ' + str((time.time() - start_time)*1000) + 'ms\n')

#function used to register messages and call the callback function
def registerMessages():

    #initialize ROS node
    rospy.init_node('registerMessages', anonymous=True)

    #create message filter subscribers for specified topics
    map_sub = message_filters.Subscriber('map', OccupancyGrid)
    pos_sub = message_filters.Subscriber('slam_out_pose', PoseStamped)

    #waits untill both subscribers recieve messages within the slop time
    ts = message_filters.ApproximateTimeSynchronizer([map_sub, pos_sub], queue_size = 3, slop = 0.1)
    
    #calls the callback function
    ts.registerCallback(callback)
    
    #spins the runction, creating a loop
    rospy.spin()

#runs the script
if __name__ == '__main__':

    #calls function to registerMessages()
    registerMessages()
