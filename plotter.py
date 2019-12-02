#! /usr/bin/env python

#script used for plotting data from tests gathered in rosbags

#importing libraries used in the script
import rospy
import rosbag
import matplotlib.pyplot as plt
import numpy as np

#function for plotting two values in one graph
def make_graph(filename):

    #make a rosbag object from the file
    bag = rosbag.Bag(filename)

    #generate lists for storing datapoints for value 1
    time1 = list()
    data1 = list()
    i = 0
    
    #read the rosbag and add data from the specified topic
    for topic, msg, t in bag.read_messages(topics='current_out_pose'):
        sec = t.to_nsec()
        time1.append((sec-1508618888979416609)*1e-9) 
        data1.append(msg.pose.orientation.z)
        i = i + 1

    #generate lists for storing datapoints for value 2
    time2 = list()
    data2 = list()
    i = 0

    #read the rosbag and add data from the specified topic
    for topic, msg, t in bag.read_messages(topics='goal_out_pose'):
        sec = t.to_nsec()
        time2.append((sec-1508618888979416609)*1e-9)
        data2.append(msg.pose.orientation.z)
        i = i + 1

    #close the rosbag
    bag.close()

    #create a subplot
    ax = plt.subplot()

    #plot the data from data1 and data2 in the subplot
    ax.plot(time1, data1, label='current', linewidth=3)
    ax.plot(time2, data2, label='wanted', linewidth=3)

    #create a legend
    legend = ax.legend(loc='center right', fontsize='x-large')

    #specify label names and font sizes for the graph
    plt.xlabel('time (seconds)', fontsize=20)
    plt.ylabel('angle (radians)', fontsize=20)
    plt.title('Current vs Wanted angles', fontsize=30)

    #set the tick parameters
    ax.tick_params(length=10, width=3)
    axi = plt.gca()
    for axis in ['top','bottom','left','right']:
        axi.spines[axis].set_linewidth(3)
    for tick in ax.xaxis.get_major_ticks():
        tick.label.set_fontsize(14)
        tick.label.set_rotation(1)
    for tick in ax.yaxis.get_major_ticks():
        tick.label.set_fontsize(14)
        tick.label.set_rotation(1)
    
    #show the graph to the user
    plt.show()

#function for plotting the difference between two values in a graph
def make_graph1(filename):

    #make a rosbag object from the file
    bag = rosbag.Bag(filename)

    #generate lists for storing datapoints for value 1
    time1 = list()
    data1 = list()
    i = 0

    #read the rosbag and add data from the specified topic
    for topic, msg, t in bag.read_messages(topics='current_out_pose'):
        sec = t.to_nsec()
        time1.append((sec-1508618888979416609)*1e-9) 
        data1.append(msg.pose.orientation.z)
        i = i + 1

    #generate lists for storing datapoints for value 2
    time2 = list()
    data2 = list()
    i = 0

    #read the rosbag and add data from the specified topic
    for topic, msg, t in bag.read_messages(topics='goal_out_pose'):
        sec = t.to_nsec()
        time2.append((sec-1508618888979416609)*1e-9)
        data2.append(msg.pose.orientation.z)
        i = i + 1

    #close the rosbag
    bag.close()

    #generate temporary list for storing the difference values
    temp = list()
    ii = 0

    #calculate the difference and add it to the temp list
    for next in data1:
        temp.append(next - data2[ii])
        ii = ii + 1

    #plot the data in the graph
    plt.plot(time1, temp, label='difference')
    plt.title('Angular difference (rad)', fontsize=30)

    #show the graph to the user
    plt.show()

#runs the script
if __name__ == '__main__':

    #set the filename of the rosbag
    filename = '/home/daniel/rosbags/avoidance_tests/unknown.bag'

    #plot the data
    make_graph(filename)