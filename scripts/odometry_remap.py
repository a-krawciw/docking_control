#!/usr/bin/env python
import rospy
import tf2_ros
import tf.transformations
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import PoseStamped, PoseWithCovariance, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


class Remapper:

    def __init__(self):
        self.tag_pose_sub = rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.handle_new_marker)
        self.tag_pub = rospy.Publisher("boat/pose", PoseWithCovarianceStamped, queue_size=10)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)


    def handle_new_marker(self, msg):
        for marker in msg.markers:

            current = PoseStamped()
            current.pose.orientation.z = 1
            current.header.frame_id = 'cg_ned'
            try:
                self.error = self.tfBuffer.transform(current, 'dock_frame')
                cov_pos = PoseWithCovarianceStamped()
                cov_pos.pose.pose = self.error
                conf = 0.000001
                cov_pos.pose.covariance[0] = conf
                cov_pos.pose.covariance[7] = conf
                cov_pos.pose.covariance[14] = conf
                cov_pos.pose.covariance[21] = conf
                cov_pos.pose.covariance[28] = conf
                cov_pos.pose.covariance[35] = conf
                self.tag_pub.publish(cov_pos)
            except:
                rospy.loginfo("Time out or disconnected tree")


def main():
    rospy.init_node('remap_marker_to_odometry')
    remap = Remapper()
    rospy.spin()

if __name__ == '__main__':
    main()