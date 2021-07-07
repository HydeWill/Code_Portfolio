import numpy

import cv2
import cv_bridge
import rospy

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image,self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.laser_cb)
        
    def laser_cb(self, laser_msg):
        #right_view < 320 < left_view
        # tmp = 999999 #
        # for i in range(639):
        #     if laser_msg.ranges[i] < tmp:
        #         tmp = laser_msg.ranges[i]

        if laser_msg.ranges[320] < 0.1:
            t = Twist()
            t.angular.z = -0.6 #turns clockwise
            self.cmd_vel_pub.publish(t)
        # laser_msg.ranges[320] < 1 and 
        if laser_msg.ranges[0] < 0.9 and laser_msg.ranges[639] <0.9:
            t = Twist()
        if laser_msg.ranges[500] < 0.9: 
            t = Twist()
            t.angular.z = -0.6 #turns clockwise
            self.cmd_vel_pub.publish(t)
        elif  laser_msg.ranges[130] < 0.9: #laser_msg.ranges[320] < 1 and
            t = Twist()
            t.angular.z = 0.6 #turns anti-clockwise
            self.cmd_vel_pub.publish(t)
        else:
            t = Twist()
            t.linear.x = 0.3 #moves forward
            self.cmd_vel_pub.publish(t)

    def image_callback(self, msg):
        cv2.namedWindow("window", 1)
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsi = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_yellow = numpy.array([10, 10, 10])
        upper_yellow = numpy.array([255, 255, 250])
        mask = cv2.inRange(hsi, lower_yellow, upper_yellow)
        h, w, d = image.shape
        left_top = h/4 
        left_bot = h/4 - 20
        left_start = w/4
        left_stop = w/4 + 20
        #mask[0:left_bot, 0:w] = 0
        #mask[left_top:h, 0:w] = 0
        mask[0:h, 0:left_start] = 0
        mask[0:h, left_stop:w] = 0
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
            err = cx - w/2
            twist = Twist()
            twist.linear.x = 0.2
            twist.angular.z = -0.001 *float(err) 
            print (twist.angular.z)

            self.cmd_vel_pub.publish(twist)
        cv2.imshow("window", image)
        cv2.waitKey(3)

    

rospy.init_node('follower')
follower = Follower()
rospy.spin()

cv2.destroyAllWindows()

