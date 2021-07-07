import rospy
from geometry_msgs.msg import Twist

cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
twist = Twist()       
#twist.linear.x = 0.2
twist.angular.z = 0.2
print (twist.angular.z)

cmd_vel_pub.publish(twist)
rospy.spin()