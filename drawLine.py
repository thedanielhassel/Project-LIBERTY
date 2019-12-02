#! /usr/bin/env python

#script used to setup and publish the marker used to visualize
#the path in RVIZ

#importing rospy and position.py
import rospy
import position as posModule

#importing message types
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

#function used to draw the line
def line_draw(nodes, mypos):

    #setup the publisher
    pub_path = rospy.Publisher('visualization_marker', Marker, queue_size=1)

    #setup the marker
    marker = Marker()
    marker.header.frame_id = "/map"
    marker.type = marker.LINE_STRIP
    marker.action = marker.ADD
    
    #set the markers scale
    marker.scale.x = 0.05
    marker.scale.y = 0.05
    marker.scale.z = 0.05

    #set the markers color
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0

    #set the markers orientation
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    #set the markers start position
    marker.pose.position.x = 0.0 
    marker.pose.position.z = 0.0

    #generate a list for the points within the marker
    marker.points = []

    #add points to the list
    if nodes != 0:
        for next in nodes:
            temp = Point()
            temp.x = next.getX()
            temp.y = next.getY()
            temp.z = 0.0
            marker.points.append(temp)

    #publish the marker to specified topic
    pub_path.publish(marker)
