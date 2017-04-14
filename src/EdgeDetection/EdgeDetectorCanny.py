#!/usr/bin/env python
import roslib
import rospy
import sys
import cv2
import cv2.cv as cv

from imutils import contours
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class cvBridge():
    def __init__(self,args):
        self.node_name = "cvBridge"        
        rospy.init_node(self.node_name)
        
        # What we do during shutdown
        rospy.on_shutdown(self.cleanup)
                
        # Create the cv_bridge object
        self.bridge = CvBridge()
        
        # Subscribe to the camera image topics and set
        # the appropriate callbacks
        self.image_sub = rospy.Subscriber(args[1], Image, self.image_callback)        
        self.image_pub = rospy.Publisher("%s/EdgeDetector/Canny" % (args[1]), Image, queue_size=10)

        rospy.loginfo("Waiting for image topics...")

    def image_callback(self, ros_image):
        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            frame = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError, e:
            print e
        
        # Convert the image to a Numpy array since most cv2 functions
        # require Numpy arrays.
        frame = np.array(frame, dtype=np.uint8)
        
        # Process the frame using the process_image() function
        preprocessing_image = self.preprocessing_image(frame)
        postprocessing_image = self.postprocessing_image(preprocessing_image)

        # Publish the processed image
        try:
            publishing_image = self.bridge.cv2_to_imgmsg(postprocessing_image, "mono8")
            self.image_pub.publish(publishing_image)
        except CvBridgeError as e:
            print(e)
                
    def preprocessing_image(self, frame):
        # Convert to greyscale
        grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Blur the image
        grey = cv2.medianBlur(grey,5)
        
	# Edge detection
        # Get Parameters from ROS interface
        MinThreshold = rospy.get_param('~MinThreshold')
        MaxThreshold = rospy.get_param('~MaxThreshold')
        edges = cv2.Canny(grey,MinThreshold,MaxThreshold)
            
        return edges

    def postprocessing_image(self, frame):
        #Image Dilation
        kernel = np.ones((3,3),np.uint8)
        dilation = cv2.dilate(frame,kernel,iterations=1)

        #Image Transformation
        kernel = np.ones((3,3),np.uint8)
        img_transform = cv2.morphologyEx(dilation, cv2.MORPH_CLOSE, kernel)
            
        return img_transform    
        
    def cleanup(self):
        print "Shutting down vision node."
        cv2.destroyAllWindows()  
    
def main(args):           
    try:
        cvBridge(args)
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down vision node."
        cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
