#!/usr/bin/env python

import numpy

import cv2
import cv_bridge
import rospy

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image,
                                          self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist,
                                           queue_size=1)
        

    def image_callback(self, msg):
        # Red_tile = numpy.array([0, 98, 100])
        # Green_tile = numpy.array([60, 98, 100])
        #Script for contours of tiles in github
        #,try and head for centre on contour?
        cv2.namedWindow("window", 1)
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsi = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_yellow = numpy.array([0, 50, 50])
        upper_yellow = numpy.array([59, 255, 255])
        mask = cv2.inRange(hsi, lower_yellow, upper_yellow)
        self.colour_rec(image,mask)
        cv2.imshow("window", image)
        cv2.waitKey(3)

    def colour_rec(self, image, mask):
        h, w, d = image.shape
        search_top = 3*h/4 
        search_bot = 3*h/4 + 20
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
            err = cx - w/2
            twist = Twist()
            twist.linear.x = 0.2
            twist.angular.z = -0.01 * float(err) 
            print (twist.angular.z)

            self.cmd_vel_pub.publish(twist)
        



#cv2.startWindowThread()
rospy.init_node('follower')
follower = Follower()
rospy.spin()

cv2.destroyAllWindows()
