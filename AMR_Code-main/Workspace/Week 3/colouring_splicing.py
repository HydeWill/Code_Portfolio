#!/usr/bin/env python

import numpy
import cv2
import cv_bridge
import rospy
from sensor_msgs.msg import Image


class image_converter:

    def __init__(self):

        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        
        # self.image_sub = rospy.Subscriber(
        #     "/camera/rgb/image_raw",
        #     Image, self.callback)

    def image_callback(self, msg):
        cv2.namedWindow("window",1)
        cv2.namedWindow("original",1)
        
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_bound = numpy.array([10, 20, 20])
        upper_bound = numpy.array([255, 255, 255])
        mask = cv2.inRange(hsv, lower_bound, upper_bound)
        cv2.bitwise_and(image, image, mask=mask)
        cv2.imshow("window", mask)
        cv2.imshow("original", image)
        cv2.waitKey(1)

# not needed in newer versions: 
# startWindowThread()
rospy.init_node('image_converter')
ic = image_converter()
rospy.spin()

# destroyAllWindows()