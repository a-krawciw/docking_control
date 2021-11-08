#!/usr/bin/env python
import math

import rospy
import tf.transformations
import tf2_ros
import tf2_geometry_msgs
from nav_msgs.msg import Path

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
        self.path_sub = rospy.Subscriber("/path", Path, self.set_path)
        self.path_pub = rospy.Publisher("path_request", PoseStamped, queue_size=10)
        self.tag_pub = rospy.Publisher("ar_tag_0", PoseStamped, queue_size=10)
        self.target = PoseStamped()
        self.error = PoseStamped()
        self.path = Path()
        self.target_state = PoseStamped()
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
            self.tag_pub.publish(marker.pose)

            current = PoseStamped()
            current.pose.orientation.z = 1
            current.header.frame_id = 'cg_ned'
            try:
                self.error = self.tfBuffer.transform(current, 'dock_frame')
                if not self._should_process():
                    self.path_pub.publish(self.error)
                self._most_recent_time = rospy.Time.now()
            except:
                rospy.loginfo("Time out or disconnected tree")

    def _should_process(self):
        return rospy.Time.now() - self._most_recent_time < rospy.Duration(1)

    def handle_boat_pos(self, msg):
        self.current_position = msg.pose

    def set_path(self, p):
        self.path = p

    def get_current_target(self):
        for waypoint in self.path.poses:
            yield waypoint

    def extract_yaw(self, q):
        return tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]


    def calc_steering_throttle(self):
        if False and self._should_process():

            position = self.error.pose.position
            target_angle = self.extract_yaw(self.target_state.pose.orientation)
            throttle = 1500
            if position.x < 0.05 and abs(position.y) < 0.1:
                rospy.loginfo("Docked")
                stage = 2
                # return 1500, 1750
            throttle = 1500 + 50 * (position.x)

            angle = self.extract_yaw(self.error.pose.orientation)
            angle_err = -target_angle + angle

            if abs(angle_err) < 0.2:
                self.target_state = self.get_current_target()
            rospy.loginfo("x:{} y{} target: {} value:{}".format(position.x, position.y, target_angle, angle))
            if throttle < 1500:
                steering = 1500 + int(350 * angle_err)
            else:
                steering = 1500 - int(350 * angle_err)
            return steering, throttle

        else:
            return 1600, 1500


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
