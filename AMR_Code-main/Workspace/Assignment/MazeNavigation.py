#Imports
import numpy
import random
import cv2
import cv_bridge
import rospy

#Publisher and Subscriber links
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class Maze_pathfinding:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        #Initialises command to publish to robot
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        #Subscription to kinect laser scanner data
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.laser_cb)
        #Subscription to kinect camera data
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image,self.image_callback)
        #Initialises variables for incoming subscription data
        self.las = 0
        self.img = 0
    
    #Callback from initial laser scanner subscription 
    def laser_cb(self, laser_msg):
        #Places data into a classwide variable
        self.las = laser_msg

    #Callback from initial camera subscription
    def image_callback(self, msg):
        #Places data into a classwide variable
        self.img = msg

    #Function to steer robot and avoid collision
    def wallAvoidance(self):
        #Central laser
        Cen_sen = self.las.ranges[320]
        #Minimum initial
        min_BR = 999
        min_FR = 999
        min_FL = 999
        min_BL = 999

        #Smallest laser range from back right region
        for i in range(0,159):
            if self.las.ranges[i] < 10:
                min_BR = min(min_BR,self.las.ranges[i])
        
        #Smallest laser range from frontright region
        for i in range(160,319):
            if self.las.ranges[i] < 10:
                min_FR = min(min_FR,self.las.ranges[i])

        #Smallest laser range from front left region
        for i in range(321,480):
            if self.las.ranges[i] < 10:
                min_FL = min(min_FL,self.las.ranges[i])

        #Smallest laser range from back left region
        for i in range(481,640):
            if self.las.ranges[i] < 10:
                min_BL = min(min_BL,self.las.ranges[i])

        #Using Min laser regions for steering and wall avoidance
        t = Twist()
        #Front sensor region movement execution 
        if Cen_sen < 0.7  and  min_FL < 0.7 and min_FR < 0.7:
            #Initialises loop time
            start = rospy.get_rostime()
            duration = rospy.Duration(3)
            #Pulishes random direction to aid exploration
            rand = random.randint(1,2)
            while (rospy.get_rostime() < start + duration):
                #print('central loop')
                if rand == 1:
                    t.angular.z = 0.5 #anti-clockwise
                    self.cmd_vel_pub.publish(t)
                else:
                    t.angular.z = -0.5 #clockwise
                    self.cmd_vel_pub.publish(t)

        #Left front region
        elif min_FL < 0.7 and Cen_sen < 0.7 :
            t.angular.z = -0.6 #clockwise
            self.cmd_vel_pub.publish(t)
            #print('Front left')

        #Right front region
        elif min_FR < 0.7 and Cen_sen < 0.7:
            t.angular.z = 0.6 #anti-clockwise
            self.cmd_vel_pub.publish(t)
            #print('Front right')

        #Leftside region
        elif min_FL < 0.6 and min_BL < 0.6:
            t.angular.z = -0.6 #clockwise
            self.cmd_vel_pub.publish(t)
            #print('Leftside')

        #Rightside
        elif min_FR < 0.65 and min_BR < 0.65:
            t.angular.z = 0.6 #anti-clockwise
            self.cmd_vel_pub.publish(t)
            #print('Rightside')

        #Too close on the rightside
        elif min_FL < 0.5 or min_BL < 0.5:
            start = rospy.get_rostime()
            duration = rospy.Duration(1)
            #Quick loop to avoid wall
            while (rospy.get_rostime() < start + duration):
                t.angular.z = -0.5 #clockwise
                self.cmd_vel_pub.publish(t)
            #print('too close left')

        #Too close on the leftside
        elif min_FR < 0.5 or min_BR < 0.5:
            start = rospy.get_rostime()
            duration = rospy.Duration(1)
            #Quick loop to avoid wall
            while (rospy.get_rostime() < start + duration):
                t.angular.z = 0.5 #anti-clockwise
                self.cmd_vel_pub.publish(t)
            #print('too close right')

        #Forward movement
        else:
            t.linear.x = 0.1 #moves forward
            self.cmd_vel_pub.publish(t)
    
    #Moves towards yellow wall line
    def wallFollower(self,image):
        #Image output window
        cv2.namedWindow("window", 1)
        #Sets parameters of mask to detect yellow line
        hsi = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_yellow = numpy.array([10, 100, 20])
        upper_yellow = numpy.array([100, 255, 250])
        mask = cv2.inRange(hsi, lower_yellow, upper_yellow)

        #Mask alterations for desired area of dectection
        h, w, d = image.shape
        mask_stop = h/4 
        mask_start = h/4 - 15
        #Zero outside of desired area
        mask[0:mask_start, 0:w] = 0
        mask[mask_stop:h, 0:w] = 0

        #Activates movement if yellow line was in desired area
        M = cv2.moments(mask)
        if M['m00'] > 0:
            #Central co-ordinates of the mask
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            #Displays a small circle where yellow was detected
            cv2.circle(image, (cx, cy), 5, (0, 0, 255), -1)
            #Pubishes Movement 
            err = cx - w/2
            twist = Twist()
            twist.linear.x = 0.1
            twist.angular.z = -0.0005 * float(err) #Creates arching movement
            #print (twist.angular.z)
            self.cmd_vel_pub.publish(twist)

        #Outputs image
        cv2.imshow("window", image)
        cv2.waitKey(3)

    #Red and Green tile recognition
    def tile_rec(self, image, mask, tile):
        #Image dimensions
        h, w, d = image.shape

        #Mask dimensions
        search_top = 7*h/8 
        search_bot = 7*h/8 + 10
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        #Converts mask to moments
        M = cv2.moments(mask)

        #If statement to preform red tile behaviour
        if M['m00'] > 0 and tile == 'red':
            #Central co-ordinates of the mask 
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            #Creates circle at centre of mask 
            cv2.circle(image, (cx, cy), 20, (255, 255, 0), -1)
            #Publishes fear behaviour
            t = Twist()
            start = rospy.get_rostime()
            duration = rospy.Duration(3)
            rand = random.randint(1,2)
            while (rospy.get_rostime() < start + duration):
                t.angular.z = 0.5 #anti-clockwise
                self.cmd_vel_pub.publish(t)
            # 
            # t.angular.z = 0
            # t.linear.x = 0

            self.cmd_vel_pub.publish(t)

        #If statement to preform green tile behaviour
        if M['m00'] > 0 and tile == 'green':
            t = Twist()
            start = rospy.get_rostime()
            duration = rospy.Duration(12)
            #12 second loop to be on top of goal
            while (rospy.get_rostime() < start + duration):
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                #Creates circle at centre of mask 
                cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
                err = cx - w/2
                t.linear.x = 0.1
                t.angular.z = -0.0001 * float(err)
                self.cmd_vel_pub.publish(t)
                #print('Green')
                rospy.sleep(0.5)
            #Goal reached
            while(True):
                #Celabration spin
                t.angular.z = 1
                self.cmd_vel_pub.publish(t)
        cv2.waitKey(3)

    def main(self):
        #Single use repositioning varibale
        start_pos = 0
        while not rospy.is_shutdown():
            if (self.las and self.img):
                #Start repositioning
                if start_pos == 0:
                    start = rospy.get_rostime()
                    duration = rospy.Duration(3)
                    t = Twist()
                    #Loops over 3 seconds for better starting pos
                    while (rospy.get_rostime() < start + duration):
                        t = Twist()
                        t.angular.z = 0.3 #anti-clockwise
                        self.cmd_vel_pub.publish(t)
                    start_pos = 1

                #Steering function call
                self.wallAvoidance()

                #Image creation for camera based functions
                image = self.bridge.imgmsg_to_cv2(self.img, desired_encoding='bgr8')
                
                #Camera based wall follower
                self.wallFollower(image)
                
                #Tile recognition
                #Red tile colour range and mask initialisation
                hsi = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
                lower_red = numpy.array([0, 50, 50])
                upper_red = numpy.array([59, 255, 255])
                red_mask = cv2.inRange(hsi, lower_red, upper_red)
                #Fears red tiles
                self.tile_rec(image,red_mask,tile = 'red')
                
                #Grenn tile colour range and mask initialisation
                lower_green = numpy.array([60,50,250])
                upper_green = numpy.array([179,255,255])
                green_mask = cv2.inRange(hsi, lower_green, upper_green)
                #Attracted to Green tile
                self.tile_rec(image,green_mask,tile = 'green')
                
        rospy.spin()
    
#Start of the program    
if __name__ =="__main__":
    #Node name 
    rospy.init_node('Maze_pathfinding')
    #Class object 
    solver = Maze_pathfinding()
    #Run main function
    solver.main()
    #Removes image display windows
    cv2.destroyAllWindows()
    

