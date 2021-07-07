import rospy
from geometry_msgs.msg import Twist

class Odom:
    
    def __init__(self):
        rospy.init_node('odom')
        self.publisher = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        

    def run(self):
        while not rospy.is_shutdown():
            t = Twist()
            t.linear.x = 0.5 # m/s
            t.angular.z = 0.5 # radians/s
            print('Moving')
            self.publisher.publish(t)
            rospy.sleep(1)
            
o = Odom()
o.run()

