#! /usr/bin/env python

#script used to send UDP messages

#import the socket library
import socket

#function used for sending messages
def sendinstructions(TARGET_IP, TARGET_PORT, MESSAGE):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(MESSAGE,(TARGET_IP, TARGET_PORT))
    
    