#! /usr/bin/env python

#script containing the decision making prosess

#importing the necessary libraries
import math

#importing the scripts sendMessage.py and directionDatabase.py
import directionDatabase as ddModule
import sendMessage as sendMessageModule

#importing the string object from std_msgs.msg
from std_msgs.msg import String

#function used to find the angle between two directions in radians
def getAngles(current, needed):
    pi = math.pi

    #calculate from quaternions to radians
    curr = math.acos(current) * 2
    need = math.acos(needed) * 2

    #get the difference
    temp = curr - need

    #make sure the difference is always the shortest way (always less than pi radians)
    if -pi < temp < pi:
        diff = temp
    else:
        if temp > 0:
            diff = temp - (2 * pi)
        if temp < 0:
            diff = temp + (2 * pi)
    return diff, need             

#function used to determine what action to take
def determine_action(path, direction, myDirection, goal, pos):

    #set IP and PORT for issuing commands
    IP = '192.168.84.254'
    PORT = 2088

    #if we are not currently at our goal
    if goal != pos:

        #if there is a path
        if path != 0:

            #genereate a direction database object
            move_dir = ddModule.check_direction()
            next_move = move_dir.getDir(direction)

            #check what direction is needed
            turn_needed, needed_angle = getAngles(myDirection, next_move)

            #if it is within the accuraccy constant
            if -0.2 < turn_needed < 0.2:

                #send instructions to drive forward
                msg = 'D' 
                sendMessageModule.sendinstructions(IP, PORT, msg)
                print('driving')
            
            #if it isn't make it turn
            else:

                #for negative turn amounts we turn right
                if turn_needed < 0:

                    #make the value needed a positive number
                    turn_abs = abs(turn_needed)
                    msg = 'R' + str(turn_abs) 

                    #send the instructions to turn right
                    sendMessageModule.sendinstructions(IP, PORT, msg)
                    print(msg)
                
                #for positive turn amounts we turn left
                else:
                    msg = 'L' + str(turn_needed)

                    #send the instructions to rutn left
                    sendMessageModule.sendinstructions(IP, PORT, msg)
                    print(msg)
            return needed_angle
                   
        #there is no path, wait for new orders or available path
        else:

            #send instructions to stop to avoid collision
            sendMessageModule.sendinstructions(IP, PORT, 'S')
            print('Searching')
            return 0.0
            
            
    #if we are at the goal
    else:

        #send instrutions to stop, we have arrived at the goal
        sendMessageModule.sendinstructions(IP, PORT, 'S')
        print('arrived')
        return 0.0
    