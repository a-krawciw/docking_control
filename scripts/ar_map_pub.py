#!/usr/bin/env python
import itertools
from collections import deque

import geometry_msgs.msg
import numpy as np
import rospy
import tf2_ros
import tf_conversions
import tf.transformations
import tf2_geometry_msgs
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
from geometry_msgs.msg import PoseStamped, TransformStamped

from contoller_utils import median_filter, vect_norm

FILTER_WIDTH = 5


def main():
    rospy.init_node("publish_map")

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    br = tf2_ros.TransformBroadcaster()

    tforms = []
    start_time = rospy.Time.now()

    def handle_ar_pose(msg):
        try:
            t = tfBuffer.lookup_transform("dock_frame", "base_link", rospy.Time(0))
            t.header.frame_id = "map"
            t.header.stamp = rospy.Time.now()
            if len(tforms) >= FILTER_WIDTH:
                del tforms[0]
            tforms.append(t)

            median_val = median_filter(tforms, lambda tr: vect_norm(tr.transform.translation))
            output_dict = {"t1":t.header.stamp.to_time(), "x1":t.transform.translation.x, "y1":t.transform.translation.y, "z1":t.transform.translation.z,
                "t2":median_val.header.stamp.to_time(), "x2":median_val.transform.translation.x, "y2":median_val.transform.translation.y, "z2":median_val.transform.translation.z}
            #rospy.loginfo(output_dict)
            br.sendTransform(median_val)
        except BaseException as e:
            print e
            rospy.logwarn("Ar_tag not in view")

    tag_pose_sub = rospy.Subscriber("ar_pose_marker", AlvarMarkers, handle_ar_pose)
    rospy.spin()


if __name__ == '__main__':
    main()
