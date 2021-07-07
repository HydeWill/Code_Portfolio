import numpy
import random
import cv2
import cv_bridge
import rospy

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class Maze_pathfinding:
    def __init__(self):
        rospy.init_node('Maze_pathfinding')
        self.bridge = cv_bridge.CvBridge()
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.laser_cb)
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image,self.image_callback)
        self.las = 0
        self.img = 0
        
    def laser_cb(self, laser_msg):
        self.las = laser_msg
        

    def image_callback(self, msg):
        self.img = msg

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
            twist.angular.z = 0
            twist.linear.x = 0
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
            #twist.angular.z = -0.01 * float(err) 
            print (twist.angular.z)

            self.cmd_vel_pub.publish(twist)
        cv2.waitKey(3)

    def wallAvoidance(self):
        Cen_sen = self.las.ranges[320]
        BL_sen = 0
        FL_sen = 0
        FR_sen = 0
        BR_sen = 0
        # BL = []
        # FL = []
        # FR = []
        # BR = []
        
        for i in range(0,159):
            if self.las.ranges[i] < 10:
                #BL.append(laser_msg.ranges[i])
                BR_sen  = BR_sen + self.las.ranges[i]
        BR_avg = BR_sen/120
        #print(backleft_avg)
        #print(laser_msg.range_max)
        #BL_min = min(BL)
        
        for i in range(160,319):
            #FL.append(laser_msg.ranges[i])
            if self.las.ranges[i] < 10:
                FR_sen  = FR_sen + self.las.ranges[i]
        FR_avg = FR_sen/120
        #FL_min = min(FL)
    
        for i in range(321,480):
            if self.las.ranges[i] < 10:
                #FR.append(laser_msg.ranges[i])
                FL_sen  = FL_sen + self.las.ranges[i]
        FL_avg = FL_sen/120
        #FR_min = min(FR)

        for i in range(481,640):
            if self.las.ranges[i] < 10:
                #BR.append(laser_msg.ranges[i])
                BL_sen  = BL_sen + self.las.ranges[i]
        BL_avg = BL_sen/120
        #BR_min = min(BR)
        t = Twist()
        if Cen_sen < 0.8  and  FL_avg < 0.8 and FR_avg < 0.8:
            start = rospy.get_rostime()
            duration = rospy.Duration(3)
            rand = random.randint(1,2)
            while (rospy.get_rostime() < start + duration):
                print('loop')
                if rand == 1:
                    t.angular.z = 0.6 #anti-clockwise
                    self.cmd_vel_pub.publish(t)
                else:
                    t.angular.z = -0.6 #clockwise
                    self.cmd_vel_pub.publish(t)

        elif FL_avg < 0.8 and Cen_sen < 0.8 :
            t.angular.z = -0.6 #clockwise
            self.cmd_vel_pub.publish(t)
            print('clock')

        elif FR_avg < 0.8 and Cen_sen < 0.8:
            t.angular.z = 0.6 #anti-clockwise
            self.cmd_vel_pub.publish(t)
            print('not clock')

        elif FL_avg < 0.8 and BL_avg < 0.8 :
            t.angular.z = -0.6 #clockwise
            self.cmd_vel_pub.publish(t)
            print('clock')

        elif FR_avg < 0.8 and BR_avg < 0.8:
            t.angular.z = 0.6 #anti-clockwise
            self.cmd_vel_pub.publish(t)
            print('not clock')
        else:
            t.linear.x = 0.2 #moves forward
            self.cmd_vel_pub.publish(t)
            
    def wallFollower(self,image):
        cv2.namedWindow("window", 1)
        hsi = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_yellow = numpy.array([10, 10, 10])
        upper_yellow = numpy.array([255, 255, 250])
        mask = cv2.inRange(hsi, lower_yellow, upper_yellow)
        h, w, d = image.shape
        left_top = h/4 
        left_bot = h/4 - 20
        left_start = w/4
        left_stop = w/4 + 20
        mask[0:left_bot, 0:w] = 0
        mask[left_top:h, 0:w] = 0
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
            err = cx - w/2
            twist = Twist()
            twist.linear.x = 0.2
            twist.angular.z = -0.001 *float(err) 
            #print (twist.angular.z)

            self.cmd_vel_pub.publish(twist)
        cv2.imshow("window", image)
        cv2.waitKey(3)
    

    def main(self):
        while not rospy.is_shutdown():
            if (self.las and self.img):
                
                self.wallAvoidance()

                
                image = self.bridge.imgmsg_to_cv2(self.img, desired_encoding='bgr8')
                
                self.wallFollower(image)
                
                #Tile recognition
                #Stops before red tile
                hsi = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
                lower_red = numpy.array([0, 50, 50])
                upper_red = numpy.array([59, 255, 255])
                red_mask = cv2.inRange(hsi, lower_red, upper_red)
                self.tile_rec(image,red_mask,tile = 'red')
                #Attracted to Green tile
                lower_green = numpy.array([60,50,250])
                upper_green = numpy.array([179,255,255])
                green_mask = cv2.inRange(hsi, lower_green, upper_green)
                self.tile_rec(image,green_mask,tile = 'green')
                
        rospy.spin()
    
    
if __name__ =="__main__":

    solver = Maze_pathfinding()
    solver.main()
    cv2.destroyAllWindows()
    

