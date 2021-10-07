#!/usr/bin/env python
import math

import rospy
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from mavros_msgs.msg import OverrideRCIn



target_dist = 0.5

def main():
    rc_pub = rospy.Publisher("mavros/rc/override", OverrideRCIn, queue_size=10)
    rospy.init_node('position_hold')

    def ar_callback(msg):
        if msg.id == 0:
            position = msg.pose.position
            rc = OverrideRCIn()
            pos_error = position.z - target_dist
            rc.channels[1] = 1500 + 450 * pos_error
            #rc.channels[1] = 1450
            angle = math.atan2(position.x, position.z)
            if rc.channels[1] < 1500:
                rc.channels[3] = 1500 + int(1000*angle)
            else:
                rc.channels[3] = 1500 - int(1000*angle)
            rospy.loginfo("{}cm: {}rad".format(position.z*100, angle))

            rc_pub.publish(rc)
            # twist.angular.z = -0.01 * angle

    rospy.Subscriber("visualization_marker", Marker, ar_callback)

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
