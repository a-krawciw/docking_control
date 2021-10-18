#!/usr/bin/env python

import rospy
from mavros_msgs.msg import OverrideRCIn

def main():
    rc_pub = rospy.Publisher("mavros/rc/override", OverrideRCIn, queue_size=10)
    rospy.init_node("override_testing")

    rc_left = 1500
    rc_right = 1500
    if rospy.has_param("~rc_left"):
        rc_left = rospy.get_param("~rc_left")
        rospy.delete_param("~rc_left")
    else:
        rospy.logwarn("No param rc_left")

    if rospy.has_param("~rc_right"):
        rc_right = rospy.get_param("~rc_right")
        rospy.delete_param("~rc_right")
    else:
        rospy.logwarn("No param rc_right")
    rc_msg = OverrideRCIn()
    rc_msg.channels[0] = rc_left
    rc_msg.channels[1] = rc_right
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rc_pub.publish(rc_msg)
        rate.sleep()
    rc_msg = OverrideRCIn()
    rc_msg.channels[0] = 1500
    rc_msg.channels[1] = 1500
    rc_pub.publish(rc_msg)

if  __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass