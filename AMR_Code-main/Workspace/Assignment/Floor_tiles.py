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
        #Script for contours of tiles in github
        #,try and head for centre on contour?
        cv2.namedWindow("window", 1)
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsi = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        #Fears Red tile
        lower_red = numpy.array([0, 50, 50])
        upper_red = numpy.array([59, 255, 255])
        red_mask = cv2.inRange(hsi, lower_red, upper_red)
        self.tile_rec(image,red_mask,tile = 'red')
        #Attracted to Green tile
        lower_green = numpy.array([60,50,250])
        upper_green = numpy.array([179,255,255])
        green_mask = cv2.inRange(hsi, lower_green, upper_green)
        self.tile_rec(image,green_mask,tile = 'green')
        
        
        cv2.imshow("window", image)
        cv2.waitKey(3)

    #Tile colour detection and reaction 
    def tile_rec(self, image, mask, tile):
        #Image dimensions
        h, w, d = image.shape

        #Mask dimensions
        search_top = 3*h/4 
        search_bot = 3*h/4 + 20
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        #Converts mask to moments
        M = cv2.moments(mask)

        ##if statement to preform red and green tile behaviour
        if M['m00'] > 0 and tile == 'red':
            #Central co-ordinates of the mask 
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            #Creates circle at centre of mask 
            cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)

            twist = Twist()
            twist.angular.z = 0.5
            #Calculates error and outputs to robot to correct to centre and move forward
            # err = cx - w/2
            # twist = Twist()
            # twist.linear.x = 0.2
            # twist.angular.z = -0.01 * float(err) 
            # print (twist.angular.z)

            self.cmd_vel_pub.publish(twist)

        if M['m00'] > 0 and tile == 'green':
            #Central co-ordinates of the mask 
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            #Creates circle at centre of mask 
            cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)

            #Calculates error and outputs to robot to correct to centre and move forward
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
