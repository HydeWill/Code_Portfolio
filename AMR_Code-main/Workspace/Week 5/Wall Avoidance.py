import rospy
import random
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class Chatter:

    def __init__(self):
        rospy.init_node('chatter')
        self.publisher = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.laser_cb)

    def laser_cb(self, laser_msg):
        # right_view < 320 < left_view
        #tmp = 999999 
        #for i in xrange(0,639):
            # if laser_msg.ranges[i] < tmp:
            #     tmp = laser_msg.ranges[i]
        Cen_sen = laser_msg.ranges[320]
        BL_sen = 0
        FL_sen = 0
        FR_sen = 0
        BR_sen = 0
        # BL = []
        # FL = []
        # FR = []
        # BR = []
        
        for i in range(0,159):
            if laser_msg.ranges[i] < 10:
                #BL.append(laser_msg.ranges[i])
                BL_sen  = BL_sen + laser_msg.ranges[i]
        BL_avg = BL_sen/120
        #print(backleft_avg)
        #print(laser_msg.range_max)
        #BL_min = min(BL)

        for i in range(160,319):
            #FL.append(laser_msg.ranges[i])
            if laser_msg.ranges[i] < 10:
                FL_sen  = FL_sen + laser_msg.ranges[i]
        FL_avg = FL_sen/120
        #FL_min = min(FL)

        for i in range(321,480):
            if laser_msg.ranges[i] < 10:
                #FR.append(laser_msg.ranges[i])
                FR_sen  = FR_sen + laser_msg.ranges[i]
        FR_avg = FR_sen/120
        #FR_min = min(FR)

        for i in range(481,640):
            if laser_msg.ranges[i] < 10:
                #BR.append(laser_msg.ranges[i])
                BR_sen  = BR_sen + laser_msg.ranges[i]
        BR_avg = BR_sen/120
        #BR_min = min(BR)

        #print(BL_avg,FL_avg,FR_avg,BR_avg)

        # if  Cen_sen < 0.6 :
        #     t = Twist()
        #     rand = random.randint(1,2)
        #     if (rand == 1):
        #         t.angular.z = 0.6 #turns clockwise
        #     elif (rand == 2):
        #         t.angular.z = -0.6 #turns anti-clockwise
        #     self.publisher.publish(t)

        if BL_avg < 0.8 and FL_avg < 0.8:
            t = Twist()
            t.angular.z = 0.6 #turns clockwise
            self.publisher.publish(t)

        elif FR_avg < 0.8 and BR_avg < 0.8:
            t = Twist()
            t.angular.z = -0.6 #turns anti-clockwise
            self.publisher.publish(t)
        
        # elif BR_avg < 0.6:
        #     t = Twist()
        #     t.angular.z = -0.4 #turns anti_clockwise
        #     self.publisher.publish(t)

        # if laser_msg.ranges[320] < 0.1:
        #     t = Twist()
        #     t.angular.z = -0.4 #turns clockwise
        #     self.publisher.publish(t)
        # # laser_msg.ranges[320] < 1 and 
        # if laser_msg.ranges[500] < 0.9: 
        #     t = Twist()
        #     t.angular.z = -0.4 #turns clockwise
        #     self.publisher.publish(t)
        # elif  laser_msg.ranges[100] < 0.9: #laser_msg.ranges[320] < 1 and
        #     t = Twist()
        #     t.angular.z = 0.4 #turns anti-clockwise
        #     self.publisher.publish(t)
        else:
            t = Twist()
            t.linear.x = 0.2 #moves forward
            self.publisher.publish(t)

    def main(self):
        rospy.spin()

c = Chatter()
c.main()