#!/usr/bin/env python
import geometry_msgs.msg
import rospy
import tf2_ros
import tf_conversions
import tf.transformations
import tf2_geometry_msgs
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
from geometry_msgs.msg import PoseStamped




def main():
     rospy.init_node("publish_map")

     tfBuffer = tf2_ros.Buffer()
     listener = tf2_ros.TransformListener(tfBuffer)

     br = tf2_ros.TransformBroadcaster()

     def handle_ar_pose(msg):
          try:
               t = tfBuffer.lookup_transform("dock_frame", "base_link", rospy.Time(0))
               t.header.frame_id = "map"
               t.header.stamp = rospy.Time.now()
               br.sendTransform(t)
          except:
               rospy.logwarn("Ar_tag not in view")
     tag_pose_sub = rospy.Subscriber("ar_pose_marker", AlvarMarkers, handle_ar_pose)
     rospy.spin()



if __name__ == '__main__':
    main()
