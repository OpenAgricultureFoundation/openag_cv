#!/usr/bin/env python
import roslib
import rospy
import sys
import cv2
import cv2.cv as cv
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class cvBridge():
    def __init__(self):
        self.node_name = "cvBridge"        
        rospy.init_node(self.node_name)
        
        # What we do during shutdown
        rospy.on_shutdown(self.cleanup)
                
        # Create the cv_bridge object
        self.bridge = CvBridge()
        
        # Subscribe to the camera image topics and set
        # the appropriate callbacks
        self.image_sub = rospy.Subscriber("/cameras/top/image_rect_color", Image, self.image_callback)        
        self.image_pub = rospy.Publisher("/cameras/top/image_rect_color/processed",Image, queue_size=1)

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
        procesed_image = self.process_image(frame)

        # Publish the processed image
        try:
            publishing_image = self.bridge.cv2_to_imgmsg(procesed_image, "mono8")
            self.image_pub.publish(publishing_image)
        except CvBridgeError as e:
            print(e)
                
    def process_image(self, frame):
        # Convert to greyscale
        grey = cv2.cvtColor(frame, cv.CV_BGR2GRAY)
        
        # Blur the image
        grey = cv2.blur(grey, (7, 7))
        
        # Compute edges using the Canny edge filter
        edges = cv2.Canny(grey, 15.0, 30.0)
        
        return edges

    def cleanup(self):
        print "Shutting down vision node."
        cv2.destroyAllWindows()  
    
def main(args):           
    try:
        cvBridge()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down vision node."
        cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
