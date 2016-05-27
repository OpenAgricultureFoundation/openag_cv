#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32

import random, numpy
import sys, getopt
import couchdb
import time

def DataPublishing(SensorName):

    for line in ch:
        doc = line['doc']
        if('variable' in doc.keys()):
                
            SensorType = doc['variable']
            SensorValueInString = doc['value']
            
            if(SensorValueInString != "error"):

                SensorValueInFloat = float(SensorValueInString)
                    
                if(SensorType == SensorName):
                    pub_air_temperature.publish(SensorValueInFloat)
                    rate.sleep()
                        
def NodeSetup():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('PFC_CouchDB_Retriever', anonymous=True)
    
def PublishersDeclarations(argv):

    SensorName = argv[0]
    
    global rate
    Frequency = int(argv[1])
    rate = rospy.Rate(Frequency) 
    
    global pub_air_temperature
    PFCName = argv[2]
    pub_air_temperature = rospy.Publisher("/PFC_%s/sensors/%s" % (PFCName,SensorName), Float32, queue_size=10)
   
def Run(argv):

    NodeSetup()
    
    # Publisher Declaration and Setup
    PublishersDeclarations(argv)
    
    # Information is obtained and processed
    SensorName = argv[0]
    DataPublishing(SensorName)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    
if __name__ == '__main__':    

    if (len(sys.argv)<6):
        print ('Not enough arguments')
        print ('Use: CouchBridge.py [HOST] [PORT] [DBNAME] [SENSORNAME] [HZ] [PFC NAME] ')

    else:
        # Command line Parameters
        HOST = sys.argv[1]
        PORT = sys.argv[2]
        DB = sys.argv[3]
        
        # Connection attempt against CouchDB Server
        couch = couchdb.Server('http://%s:%s/'%(HOST,PORT))

        # If we can reach the server
        if(couch.version()):

            print('Connected to %s in port %s'%(HOST,PORT))

            # We retrieve database info
            db = couch[DB]
            
            # We declare a continuous change callback
            ch = db.changes(feed="continuous",since='now',heartbeat='1000',include_docs=True)

            # Main publisher's function
            Run(sys.argv[4:])
            
        # If not ...
        else:
            print('Can not reach %s in port %s'%(HOST,PORT))
