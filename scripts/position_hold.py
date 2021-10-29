#!/usr/bin/env python
import math

import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, Pose
from ar_track_alvar_msgs.msg import AlvarMarkers

from mavros_utils import RCManager


def rc_callback(msg):
    if msg.channels[6] < 1200:
        global ros_mode
        ros_mode = False



class RouteFinder:

    def __init__(self, tf2_buffer=None):
        self.boat_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.handle_boat_pos)
        self.tag_pose_sub = rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.handle_new_marker)
        self.tag_pub = rospy.Publisher("ar_tag_0", PoseStamped, queue_size=10)
        self.target = PoseStamped()
        self.error = PoseStamped()
        self.target.header.frame_id = 'map'
        self.current_position = Pose()
        self._most_recent_time = rospy.Time.now() - rospy.Duration(11)
        if tf2_buffer is None:
            self.tfBuffer = tf2_ros.Buffer()
            self.listener = tf2_ros.TransformListener(self.tfBuffer)
        else:
            self.tfBuffer = tf2_buffer

    def handle_new_marker(self, msg):
        for marker in msg.markers:
            if marker.id == 3:
                self.target = marker.pose
                self.target.header = marker.header
                self.target.header.frame_id = self.target.header.frame_id.replace("/", "")
                self.tag_pub.publish(marker.pose)
                self._most_recent_time = rospy.Time.now()

    def _should_process(self):
        return rospy.Time.now() - self._most_recent_time < rospy.Duration(10)

    def handle_boat_pos(self, msg):
        self.current_position = msg.pose
        if self._should_process():
            try:
                self.target.header.stamp = rospy.Time.now() - rospy.Duration(1)
                self.error = self.tfBuffer.transform(self.current_position, 'ar_marker_3')
            except:
                rospy.loginfo("Time out or disconnected tree")

    def calc_steering_throttle(self):
        if self._should_process():
            position = self.target.pose.position
            throttle = 1500 + 450 * (position.x-1)
            angle = math.atan2(position.y, position.x)
            rospy.loginfo("x:{} y{}".format(position.x, position.y))
            if throttle < 1500:
                steering = 1500 + int(500 * angle)
            else:
                steering = 1500 - int(500 * angle)
            return steering, throttle
        else:
            return 1500, 1500


def main():
    rospy.init_node('position_hold')

    tfBuffer = tf2_ros.Buffer(rospy.Duration(20))
    tf_listener = tf2_ros.TransformListener(tfBuffer)
    boat = RouteFinder(tfBuffer)
    rc_controller = RCManager()

    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        rc_controller.setSteerThrottle(*boat.calc_steering_throttle())
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
