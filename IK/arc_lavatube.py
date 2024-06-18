import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


def laser_clbk(msg):
    vel = Twist()
    left_extreme = msg.ranges[0:143]
    left = msg.ranges[144:287]
    center = msg.ranges[288:431]
    right = msg.ranges[432:575]
    right_extreme = msg.ranges[576:719]
    width_lavatube = 5

    if min(right) > min(left):
        vel.angular.z = 0.5
        vel.linear.x = 1
    elif min(left) > min(right):
        vel.angular.z = -0.5
        vel.linear.x = 1
    elif min(left_extreme)+min(right_extreme) > width_lavatube:
        vel.angular.z = 0.0
        vel.linear.x = 0.0
        print("exited lavatube succesfully and rover stopped")

    pub.publish(vel)


if __name__ == "__main__":
    rospy.init_node("lavatube", anonymous=True)
    rospy.Subscriber('/galileo/laser_scan', LaserScan, laser_clbk)
    pub = rospy.Publisher('/galileo/cmd_vel', Twist, queue_size=10)
    rospy.spin()
