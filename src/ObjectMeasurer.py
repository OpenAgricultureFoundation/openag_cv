#!/usr/bin/env python

# import the necessary packages
import roslib
import rospy
import sys
import numpy as np
import argparse
import imutils
import cv2
import cv2.cv as cv

from scipy.spatial import distance as dist
from imutils import perspective
from imutils import contours

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

class CV():
    def __init__(self, argv):
        self.node_name = "ObjectMeasurer"        
        rospy.init_node(self.node_name)
        
        # What we do during shutdown
        rospy.on_shutdown(self.cleanup)
                
        # Create the cv_bridge object
        self.bridge = CvBridge()
        
        # Subscribe to the camera image topics and set
        # the appropriate callbacks
        self.image_sub = rospy.Subscriber(argv[1], Image, self.image_callback)        
        self.image_pub = rospy.Publisher("%s/ObjectMeasurer" % (argv[1]), Image, queue_size=10)

        rospy.loginfo("Waiting for image topics...")

    def midpoint(self, ptA, ptB):
	return ((ptA[0] + ptB[0]) * 0.5, (ptA[1] + ptB[1]) * 0.5)
        
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
            publishing_image = self.bridge.cv2_to_imgmsg(procesed_image, "bgr8")
            self.image_pub.publish(publishing_image)
        except CvBridgeError as e:
            print(e)
                
    def process_image(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (7, 7), 0)

        # perform edge detection, then perform a dilation + erosion to
        # close gaps in between object edges
        edged = cv2.Canny(gray, 45, 100)
        edged = cv2.dilate(edged, None, iterations=1)
        edged = cv2.erode(edged, None, iterations=1)

        # find contours in the edge map
        cnts = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL,
	                        cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if imutils.is_cv2() else cnts[1]

        # sort the contours from left-to-right and initialize the
        # 'pixels per metric' calibration variable
        (cnts, _) = contours.sort_contours(cnts)

        pixelsPerMetric = None

        # compute the rotated bounding box of the contour
        processed_frame = frame.copy()

        # loop over the contours individually
        for c in cnts:
                # if the contour is not sufficiently large, ignore it
                if cv2.contourArea(c) < 100:
	                continue

                # compute the rotated bounding box of the contour
                box = cv2.minAreaRect(c)
                box = cv2.cv.BoxPoints(box) if imutils.is_cv2() else cv2.boxPoints(box)
                box = np.array(box, dtype="int")
    
                # order the points in the contour such that they appear
                # in top-left, top-right, bottom-right, and bottom-left
                # order, then draw the outline of the rotated bounding
                # box
                box = perspective.order_points(box)
                cv2.drawContours(processed_frame, [box.astype("int")], -1, (0, 255, 0), 2)
    
                # loop over the processed_frame points and draw them
                for (x, y) in box:
                        cv2.circle(processed_frame, (int(x), int(y)), 5, (0, 0, 255), -1)
            
                # unpack the ordered bounding box, then compute the midpoint
                # between the top-left and top-right coordinates, followed by
                # the midpoint between bottom-left and bottom-right coordinates
                (tl, tr, br, bl) = box
                (tltrX, tltrY) = self.midpoint(tl, tr)
                (blbrX, blbrY) = self.midpoint(bl, br)
                        
                # compute the midpoint between the top-left and top-right points,
                # followed by the midpoint between the top-righ and bottom-right
                (tlblX, tlblY) = self.midpoint(tl, bl)
                (trbrX, trbrY) = self.midpoint(tr, br)
                
                # draw the midpoints on the frame
                cv2.circle(processed_frame, (int(tltrX), int(tltrY)), 5, (255, 0, 0), -1)
                cv2.circle(processed_frame, (int(blbrX), int(blbrY)), 5, (255, 0, 0), -1)
                cv2.circle(processed_frame, (int(tlblX), int(tlblY)), 5, (255, 0, 0), -1)
                cv2.circle(processed_frame, (int(trbrX), int(trbrY)), 5, (255, 0, 0), -1)
                
                # draw lines between the midpoints
                cv2.line(processed_frame, (int(tltrX), int(tltrY)), (int(blbrX), int(blbrY)),
	                 (255, 0, 255), 2)
                cv2.line(processed_frame, (int(tlblX), int(tlblY)), (int(trbrX), int(trbrY)),
	                 (255, 0, 255), 2)

                # compute the Euclidean distance between the midpoints
                dA = dist.euclidean((tltrX, tltrY), (blbrX, blbrY))
                dB = dist.euclidean((tlblX, tlblY), (trbrX, trbrY))

                # if the pixels per metric has not been initialized, then
                # compute it as the ratio of pixels to supplied metric
                # (in this case, inches)
                if pixelsPerMetric is None:
	                pixelsPerMetric = dB / 2

                # compute the size of the object
                dimA = dA / pixelsPerMetric
                dimB = dB / pixelsPerMetric

                # draw the object sizes on the frame
                cv2.putText(processed_frame, "{:.1f}in".format(dimA),
		            (int(tltrX - 15), int(tltrY - 10)), cv2.FONT_HERSHEY_SIMPLEX,
		            0.65, (255, 255, 255), 2)
                cv2.putText(processed_frame, "{:.1f}in".format(dimB),
		            (int(trbrX + 10), int(trbrY)), cv2.FONT_HERSHEY_SIMPLEX,
		            0.65, (255, 255, 255), 2)

        return processed_frame

    def cleanup(self):
        print "Shutting down vision node."
        cv2.destroyAllWindows()  

def main(args):           
    try:
        CV(args)
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down vision node."
        cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
