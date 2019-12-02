#! /usr/bin/env python

#script used to send stop instructions to NODE-RED
#Used only if main.py fails to stop the rover or if 
#something goes wrong

#imports necessary libraries and scripts
import socket
import sendMessage as sm 

#reuns the script at start
if __name__ == '__main__':

    #specify IP, PORT and MESSAGE
    IP = '192.168.84.254'
    PORT = 2088
    MESSAGE = 'S0.232'

    #uses function from sendMessage.py to send messages
    #to NODE-RED
    sm.sendinstructions(IP, PORT, MESSAGE)