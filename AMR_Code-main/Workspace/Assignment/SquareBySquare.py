import rospy

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class SquareBySquare:
    def __init__(self):
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image,
                                          self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist,
                                           queue_size=1)
    
    def callback(self,msg):


rospy.init_node('Squares')
Squares = SquareBySquare()
rospy.spin()
