#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool

import random, numpy
import sys, getopt
import requests
import json

def callback(data, Parameters):

    HOST = Parameters[0]
    PORT = Parameters[1]
    ACTUATORNAME = Parameters[2]
    
    msg = DataConverter(data)
    ActuatorState = {'state': msg}

    r = requests.post('http://%s:%s/_openag/%s/set_state'%(HOST,PORT,ACTUATORNAME),\
                      data=json.dumps(ActuatorState),\
                      headers={"Content-Type": "application/json"})

def DataConverter(data):

    if(data.data):
        msg = 'true'
    else:
        msg = 'false'

    return msg
    
def Subscriber(argv):

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('PFC_CouchDB_Feeder', anonymous=True)

    # Topic you want to subscribe
    rospy.Subscriber(argv[0], Bool, callback, argv[1:])
    
def Run(argv):

    # Information is obtained and processed
    Subscriber(argv)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    
if __name__ == '__main__':
    
    if (len(sys.argv)<4):
        print ('Not enough arguments')
        print ('Use : CouchFeeder.py [TOPIC] [HOST] [PORT] [ACTUATOR NAME]')
    else:
        Run(sys.argv[1:])
