import rospy
from math import radians
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

wheel_radius = .06
robot_radius = .2

class Kin:
    def __init__(self):
        self.Kin_sub = rospy.Subscriber("/wheel_vel_left", Float32, self.Kin_callback)
        self.Wheel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        
    
    def Kin_callback(self,l_wheel):
        left_wheel_vel = l_wheel.data
        right_wheel_vel =0.0
        v,a = forward_kinematics(left_wheel_vel,right_wheel_vel)
        t = Twist()
        t.linear.x = v
        t.angular.z = a
        self.Wheel_pub.publish(t)




# computing the forward kinematics for a differential drive
def forward_kinematics(w_l, w_r):
    c_l = wheel_radius * w_l
    c_r = wheel_radius * w_r
    v = (c_l + c_r) / 2
    a = (c_r - c_l) / (2 * robot_radius)
    return (v, a)


# computing the inverse kinematics for a differential drive
def inverse_kinematics(v, a):
    c_l = v - (robot_radius * a)
    c_r = v + (robot_radius * a)
    w_l = c_l / wheel_radius
    w_r = c_r / wheel_radius
    return (w_l, w_r)


# inverse kinematics from a Twist message (This is what a ROS robot has to do)
def inverse_kinematics_from_twist(t):
    return inverse_kinematics(t.linear.x, t.angular.z)

# if __name__ == "__main__":

    # (w_l, w_r) = inverse_kinematics(0.0, 1.0)
    # print "w_l = %f,\tw_r = %f" % (w_l, w_r)

    # (v, a) = forward_kinematics(w_l, w_r)
    # print "v = %f,\ta = %f" % (v, a)

    # from geometry_msgs.msg import Twist
    # t = Twist()

    # t.linear.x = 0.3
    # t.angular.z = 0.8

    # (w_l, w_r) = inverse_kinematics_from_twist(t)
    # print "w_l = %f,\tw_r = %f" % (w_l, w_r)


rospy.init_node('kin')
Kin()
rospy.spin()