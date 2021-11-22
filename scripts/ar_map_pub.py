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

from contoller_utils import extract_pitch



FILTER_WIDTH = 5

def main():
     rospy.init_node("publish_map")

     tfBuffer = tf2_ros.Buffer()
     listener = tf2_ros.TransformListener(tfBuffer)

     br = tf2_ros.TransformBroadcaster()

     tforms = deque()

     def handle_ar_pose(msg):
          try:
               t = tfBuffer.lookup_transform("dock_frame", "base_link", rospy.Time(0))
               t.header.frame_id = "map"
               t.header.stamp = rospy.Time.now()

               if abs(extract_pitch(t.transform.rotation)) < np.pi/2:
                    br.sendTransform(t)
          except:
               rospy.logwarn("Ar_tag not in view")
     tag_pose_sub = rospy.Subscriber("ar_pose_marker", AlvarMarkers, handle_ar_pose)
     rospy.spin()



if __name__ == '__main__':
    main()
