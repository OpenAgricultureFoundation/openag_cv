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

    def __init__(self, args):
        self.node_name = "MaskPlantTray"
        rospy.init_node(self.node_name)

        # What we do during shutdown
        rospy.on_shutdown(self.cleanup)

        # Create the cv_bridge object
        self.bridge = CvBridge()

        # Subscribe to the camera image topics and set
        # the appropriate callbacks
        self.image_sub = rospy.Subscriber(args[1], Image, self.image_callback)
        self.image_pub = rospy.Publisher(
            "%s/MaskPlantTray" % (args[1]), Image, queue_size=10)

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
        postprocessing_image = self.preprocessing_image(frame)

        # Publish the processed image
        try:
            publishing_image = self.bridge.cv2_to_imgmsg(
                postprocessing_image, "bgr8")
            self.image_pub.publish(publishing_image)
        except CvBridgeError as e:
            print(e)

    def preprocessing_image(self, frame):
        height, width = frame.shape[:2]
        mask = np.zeros((height, width), np.uint8)

        cv2.rectangle(mask, (width / 3 + 10, height / 3 - 130),
                      (width * 2 / 3 + 10, height * 2 / 3 - 10), [255, 255, 255], thickness=-1)

        masked_img = cv2.bitwise_and(frame, frame, mask=mask)
        return masked_img

    def postprocessing_image(self, frame):
        return frame

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
