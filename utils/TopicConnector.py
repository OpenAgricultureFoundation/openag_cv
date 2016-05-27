#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32, Float64
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Int8, Int16, Int32

import random, numpy
import sys, getopt
    
def callback(data):
    # Publish info into Topic
    rospy.loginfo(data.data)
    pub.publish(data.data)

def Subscriber(SubscribingTopicName,SubscribingMsgType):

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('Topic_Connection', anonymous=True)

    # Topic you want to subscribe
    rospy.Subscriber(SubscribingTopicName, SubscribingMsgType, callback)
    
def Publisher(PublishingTopicName,PublishingMsgType):

    global pub
    pub = rospy.Publisher(PublishingTopicName, PublishingMsgType, queue_size=10)
    
def Run(argv):

    # Information is obtained and processed
    argv[0] = SubscribingTopicName
    argv[1] = SubscribingMsgType
    Subscriber(SubscribingTopicName,SubscribingMsgType)

    # Information is published
    argv[2] = PublishingTopicName
    argv[3] = PublishingMsgType
    Publisher(PublishingTopicName,PublishingMsgType)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    
if __name__ == '__main__':
    
    if (len(sys.argv)<4):
        print ('Not enough arguments')
        print ('USE: TopicConnector.py [TOPIC TO SUBSCRIBE] [MSG TYPE] [TOPIC TO PUBLISH] [MSG TYPE]')
    else:
        Run(sys.argv[1:])
