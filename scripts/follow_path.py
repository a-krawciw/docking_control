#!/usr/bin/env python
import math
import threading

import numpy as np

import rospy
import tf.transformations
import tf2_ros
import tf2_geometry_msgs
from nav_msgs.msg import Path

from geometry_msgs.msg import PoseStamped, Pose
from ar_track_alvar_msgs.msg import AlvarMarkers

from mavros_utils import RCManager, BoatConfig

import contoller_utils as utils


def rc_callback(msg):
    if msg.channels[6] < 1200:
        global ros_mode
        ros_mode = False


class RouteFinder:

    def __init__(self, tf2_buffer=None):
        self.boat_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.handle_boat_pos)
        self.tag_pose_sub = rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.handle_new_marker)
        self.path_sub = rospy.Subscriber("/path", Path, self.set_path)
        self.path_pub = rospy.Publisher("path_request", PoseStamped, queue_size=10)
        self.target = PoseStamped()
        self.error = PoseStamped()
        self.path = Path()
        self.path_lock = threading.Lock()
        self.target.header.frame_id = 'map'
        self.current_position = Pose()
        self._most_recent_time = rospy.Time.now() - rospy.Duration(11)
        self.stage = 0
        if tf2_buffer is None:
            self.tfBuffer = tf2_ros.Buffer()
            self.listener = tf2_ros.TransformListener(self.tfBuffer)
        else:
            self.tfBuffer = tf2_buffer

    def handle_new_marker(self, msg):
        for marker in msg.markers:
            self.target = marker.pose
            self.target.header = marker.header
            self.target.header.frame_id = self.target.header.frame_id.replace("/", "")

            current = PoseStamped()
            current.pose.orientation.w = 1
            current.header.frame_id = 'cg_ned'
            try:
                self.error = self.tfBuffer.transform(current, 'dock_frame')
                if len(self.path.poses) < 1 and rospy.Time.now() - self._most_recent_time > rospy.Duration(10):
                    self.path_pub.publish(self.error)
                    rospy.loginfo("Requesting new path")
                    self.path.header.stamp = rospy.Time.now()
                self._most_recent_time = rospy.Time.now()
            except:
                rospy.loginfo("Time out or disconnected tree")

    def _should_process(self):
        return rospy.Time.now() - self._most_recent_time < rospy.Duration(1)

    def handle_boat_pos(self, msg):
        self.current_position = msg.pose

    def set_path(self, p):
        self.path = p

    def calc_force_torque(self):
        if self._should_process():

            position = self.error.pose.position
            try:
                (dist, idx) = utils.min_dist(self.path, self.error)
            except (ValueError, IndexError):
                return 0, 0
            target_angle = utils.extract_yaw(self.path.poses[idx].pose.orientation)
            path_pos = self.path.poses[idx].pose
            path_pos = np.array([path_pos.position.x, path_pos.position.y, target_angle])
            current_theta = utils.extract_yaw(self.error.pose.orientation)
            rospy.loginfo("target {}".format(path_pos))
            look_ahead_point = np.array(path_pos[0:2] + 0.05 * np.array([np.cos(target_angle), np.sin(target_angle)]))
            rospy.loginfo("Lookahead point {}".format(look_ahead_point))

            restoring_angle = np.arctan2(look_ahead_point[1] - position.y, look_ahead_point[0] - position.x)

            angle_err = 5 * utils.theta_diff(current_theta, target_angle) + \
                        2 * utils.theta_diff(current_theta, restoring_angle)

            T = 0.4 * angle_err / np.pi * 0.75 * 0.165
            F = 0.75/(1-dist)*0.75
            F = 0.1

            if position.x < 0.05 and abs(position.y) < 0.1:
                rospy.loginfo("Docked")
                return 0.75, 0

            rospy.loginfo("x:{} y{} target: {} current:{}".format(position.x, position.y, np.rad2deg(restoring_angle),
                                                                  np.rad2deg(current_theta)))
            return F, T

        else:
            return 0, 0


def main():
    rospy.init_node('position_hold')

    tfBuffer = tf2_ros.Buffer(rospy.Duration(20))
    tf_listener = tf2_ros.TransformListener(tfBuffer)
    boat = RouteFinder(tfBuffer)
    rc_controller = RCManager()
    rc_controller.set_raw_override()
    boat_config = BoatConfig(0.165, -0.0123/4, -80)

    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        rc_controller.setForceTorque(*boat.calc_force_torque(), boat_config=boat_config)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
