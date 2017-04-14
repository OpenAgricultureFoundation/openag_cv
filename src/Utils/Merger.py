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
        self.node_name = "ImgMerger"
        rospy.init_node(self.node_name)

        # What we do during shutdown
        rospy.on_shutdown(self.cleanup)

        # Create the cv_bridge object
        self.bridge = CvBridge()

        self.frame_img1 = np.zeros((1280, 1024), np.uint8)
        self.frame_img2 = np.zeros((1280, 1024), np.uint8)

        # Subscribe to the camera image topics and set
        # the appropriate callbacks
        self.image_sub_first_image = rospy.Subscriber(
            args[1], Image, self.image_callback_img1)
        self.image_sub_second_image = rospy.Subscriber(
            args[2], Image, self.image_callback_img2)

        self.image_pub = rospy.Publisher(args[3], Image, queue_size=10)

        rospy.loginfo("Waiting for image topics...")

    def image_callback_img1(self, ros_image):
        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            frame = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError, e:
            print e

        # Convert the image to a Numpy array since most cv2 functions
        # require Numpy arrays.
        self.frame_img1 = np.array(frame, dtype=np.uint8)

    def image_callback_img2(self, ros_image):
        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            frame = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError, e:
            print e

        # Convert the image to a Numpy array since most cv2 functions
        # require Numpy arrays.
        self.frame_img2 = np.array(frame, dtype=np.uint8)

        self.merge_imgs(self.frame_img1, self.frame_img2)

    def merge_imgs(self, frame_img1, frame_img2):
        # Process the frame using the process_image() function
        postprocessing_image = self.preprocessing_image(frame_img1, frame_img2)

        # Publish the processed image
        try:
            publishing_image = self.bridge.cv2_to_imgmsg(
                postprocessing_image, "bgr8")
            self.image_pub.publish(publishing_image)
        except CvBridgeError as e:
            print(e)

    def preprocessing_image(self, frame_img1, frame_img2):
        merged_img = cv2.addWeighted(frame_img1, 0.7, frame_img2, 0.3, 0)
        return merged_img

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
