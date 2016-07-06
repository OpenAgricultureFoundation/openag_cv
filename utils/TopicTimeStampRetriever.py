#!/usr/bin/env python

import rosbag
import sys, getopt


def Run(argv):

    bag = rosbag.Bag(argv[0])

    for topic, msg, t in bag.read_messages(topics=[argv[1]]):
        print msg.header.stamp

    bag.close()
        
if __name__ == '__main__':

    if (len(sys.argv)<3):
        print ('Not enough arguments')
        print ('USE: TimeStampRetriever.py [BAGFILE] [TOPIC]')
    else:
        Run(sys.argv[1:])


