#!/usr/bin/env python
import roslib
import rospy
import sys
import cv2
import cv2.cv as cv

from imutils import contours, perspective
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
        self.image_pub = rospy.Publisher("%s/EdgeDetector/AdaptiveThreshold" % (args[1]), Image, queue_size=10)

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
        postprocessing_image = self.processing(frame)

        # Publish the processed image
        try:
            publishing_image = self.bridge.cv2_to_imgmsg(postprocessing_image, "mono8")
            self.image_pub.publish(publishing_image)
        except CvBridgeError as e:
            print(e)
                

    def processing(self, img_rgb):
        numDownSamples = 2       # number of downscaling steps
        numBilateralFilters = 7  # number of bilateral filtering steps

        # -- STEP 1 --
        # downsample image using Gaussian pyramid
        img_color = img_rgb
        for _ in xrange(numDownSamples):
            img_color = cv2.pyrDown(img_color)

        # repeatedly apply small bilateral filter instead of applying
        # one large filter
        for _ in xrange(numBilateralFilters):
            img_color = cv2.bilateralFilter(img_color, 9, 9, 7)

        # upsample image to original size
        for _ in xrange(numDownSamples):
            img_color = cv2.pyrUp(img_color)

        # -- STEPS 2 and 3 --
        # convert to grayscale and apply median blur
        img_gray = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2GRAY)
        img_blur = cv2.medianBlur(img_gray, 7)

        # -- STEP 4 --
        # detect and enhance edges
        img_edge = cv2.adaptiveThreshold(img_blur, 255,
                                         cv2.ADAPTIVE_THRESH_MEAN_C,
                                         cv2.THRESH_BINARY, 9, 2)
        return img_edge
    
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
