#!/usr/bin/env python

import tf2_ros
import rospy
from geometry_msgs.msg import TwistStamped

class VelocityConvertor:

    def __init__(self, rate=2):
        self.tfBuffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tfBuffer)
        self.pose_sub = rospy.Subscriber("mavros/local_position/velocity_body", TwistStamped, self._convert)
        self.pub = rospy.Publisher("cg_frame/velocity_body", TwistStamped, queue_size=10)

    def _convert(self, twist_msg):
        try:
            (frame, orientation) = self.tfBuffer.lookup_transform("base_link", "cg_ned", rospy.Time.now(), rospy.Duration(0, 10000))

            #self.pub.publish(conv)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(e)


if __name__ == '__main__':
    rospy.init_node('tf2_boat_listener')
    v = VelocityConvertor()
    rospy.spin()

