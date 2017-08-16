#!/usr/bin/env python
import roslib
import rospy
import sys
import cv2
import cv2.cv as cv

from imutils import contours
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from scipy.spatial import distance
import numpy as np

from matplotlib import pyplot as plt


class cvBridge():

    def __init__(self, args):
        self.node_name = "cvBridge"
        rospy.init_node(self.node_name)

        # What we do during shutdown
        rospy.on_shutdown(self.cleanup)

        # Create the cv_bridge object
        self.bridge = CvBridge()

        # Subscribe to the camera image topics and set
        # the appropriate callbacks
        self.image_sub = rospy.Subscriber(args[1], Image, self.image_callback)
        self.image_pub = rospy.Publisher(
            "%s/BlobDetector" % (args[1]), Image, queue_size=10)

        # Detector
        # Set up the detector with parameters.

        # Setup SimpleBlobDetector parameters.
        params = cv2.SimpleBlobDetector_Params()

        # Filter by Color.
        params.filterByColor = rospy.get_param('~FilterByColor')
        params.blobColor = rospy.get_param('~BlobColor')

        # Filter by Area.
        params.filterByArea = rospy.get_param('~FilterByArea')
        params.maxArea = rospy.get_param('~BlobMaxArea')

        # Filter by Circularity
        params.filterByCircularity = rospy.get_param('~FilterByCircularity')
        params.minCircularity = rospy.get_param('~BlobMinCircularity')
        params.maxCircularity = rospy.get_param('~BlobMaxCircularity')

        # Filter by Convexity
        params.filterByConvexity = rospy.get_param('~FilterByConvexity')
        params.minConvexity = rospy.get_param('~BlobMinConvexity')
        params.maxConvexity = rospy.get_param('~BlobMaxConvexity')

        # Filter by Inertia
        params.filterByInertia = rospy.get_param('~FilterByInertia')
        params.minInertiaRatio = rospy.get_param('~BlobMinInertia')
        params.maxInertiaRatio = rospy.get_param('~BlobMaxInertia')

        self.MaxLeavesSocketA = [0]
        self.MaxLeavesSocketB = [0]
        self.MaxLeavesSocketC = [0]
        self.MaxLeavesSocketD = [0]
        self.MaxLeavesSocketE = [0]
        self.MaxLeavesSocketF = [0]

        self.detector = cv2.SimpleBlobDetector(params)
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
            publishing_image = self.bridge.cv2_to_imgmsg(
                postprocessing_image, "bgr8")
            self.image_pub.publish(publishing_image)
        except CvBridgeError as e:
            print(e)

    def k_means(self, keypoints):
        # convert to np.float32
        X = [k.pt[0] for k in keypoints]
        Y = [k.pt[1] for k in keypoints]

        Z = np.float32(np.vstack((X, Y)).T)

        # define criteria and apply kmeans()
        criteria = (cv2.TERM_CRITERIA_EPS +
                    cv2.TERM_CRITERIA_MAX_ITER, 25, 1.0)

        K = 6

        ret, label, center = cv2.kmeans(
            Z, K, criteria, 25, cv2.KMEANS_RANDOM_CENTERS)

        Leaves = {}
        for x in range(K):
            Leaves['Center%d' % x] = Z[label.ravel() == x]

        Centers = {}
        for x in range(K):
            Centers['Center%d' % x] = center[
                label.flatten()][label.ravel() == x][0, :]

        ReferencePoint = np.array([0, 0])
        print Centers.values()

        Distances = {}
        for x in range(K):
            Distances['Center%d' % x] = distance.euclidean(
                ReferencePoint.ravel(), Centers['Center%d' % x])

        CentersOrderedList = sorted(Distances.items(), key=lambda x: x[1])

        leaves_data = {'SocketA': [Leaves[CentersOrderedList[0][0]], Centers[CentersOrderedList[0][0]], self.MaxLeavesSocketA],
                       'SocketB': [Leaves[CentersOrderedList[1][0]], Centers[CentersOrderedList[1][0]], self.MaxLeavesSocketB],
                       'SocketC': [Leaves[CentersOrderedList[2][0]], Centers[CentersOrderedList[2][0]], self.MaxLeavesSocketC],
                       'SocketD': [Leaves[CentersOrderedList[3][0]], Centers[CentersOrderedList[3][0]], self.MaxLeavesSocketD],
                       'SocketE': [Leaves[CentersOrderedList[4][0]], Centers[CentersOrderedList[4][0]], self.MaxLeavesSocketE],
                       'SocketF': [Leaves[CentersOrderedList[5][0]], Centers[CentersOrderedList[5][0]], self.MaxLeavesSocketF]}

        return leaves_data

    def print_number_of_leaves(self, frame, leaves_data):
        for key, value in leaves_data.iteritems():
            # Calculate the number of leaves depending on the coordinates
            # available in the leaves dictionary
            number_of_leaves = (len(value[0]))

            if number_of_leaves > leaves_data[key][2][-1]:
                leaves_data[key][2].append(number_of_leaves)

            if number_of_leaves >= 4:
                cv2.putText(frame, "Leaves:{:.1f}".format(leaves_data[key][2][-1]),
                            (int(leaves_data[key][1][0] - 150), int(leaves_data[key][1][1] + 50)
                             ), cv2.FONT_HERSHEY_SIMPLEX,
                            0.65, (255, 0, 0), 2)

        # for key, value in leaves_data.iteritems():
        #     print leaves_data[key][2]

        # print "--------------"

        return frame

    def preprocessing_image(self, frame):
        return frame

    def postprocessing_image(self, frame):
        # Detect blobs.
        keypoints = self.detector.detect(frame)

        # Draw detected blobs as red circles.
        # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the
        # circle corresponds to the size of blob
        im_with_keypoints = cv2.drawKeypoints(frame,
                                              keypoints,
                                              np.array([]),
                                              (0, 0, 255),
                                              cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        leaves_data = self.k_means(keypoints)
        frame = self.print_number_of_leaves(im_with_keypoints, leaves_data)
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
