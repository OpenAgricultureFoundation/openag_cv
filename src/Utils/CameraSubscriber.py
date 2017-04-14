#!/usr/bin/env python

import rospy
import sys, getopt
from sensor_msgs.msg import Image

global CurrentImage

def callback_timer(event):
    pub.publish(CurrentImage)

def callback(data):
    global CurrentImage
    CurrentImage = data
        
def Subscriber(argv):
    
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('CameraSubscriber', anonymous=True)

    # Topic you want to subscribe
    rospy.Subscriber(argv[1], Image, callback)
    
def Publisher(argv):

    global pub
    pub = rospy.Publisher("%s_shot" % (argv[1]), Image, queue_size=1)
   
def Run(argv):
    
    # Information is published
    Publisher(argv)

    # Information about the Subscribing Topic
    Subscriber(argv)

    rospy.Timer(rospy.Duration(int(argv[0]),0), callback_timer)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    
if __name__ == '__main__':    
    Run(sys.argv[1:])
