#!/usr/bin/env python
import math

import rospy
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker

twist = Twist()

def ar_callback (msg):
    if msg.id == 0:
        position = msg.pose.position
        twist.linear.x = 0.54
        angle = math.atan2(position.x, position.z)
        twist.angular.z = -0.01 * angle
        if angle < 0:
            rospy.loginfo("Turn right")
        else:
            rospy.loginfo("Turn left")

def main():
    pub = rospy.Publisher('setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)
    rospy.init_node('steering_raw')
    rospy.Subscriber("visualization_marker", Marker, ar_callback)
    
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        global twist
        pub.publish(twist)
        twist = Twist()
        rate.sleep()


if  __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
