#!/usr/bin/env python
import math

import rospy
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
from ar_track_alvar_msgs.msg import AlvarMarkers
import tf2_ros
import tf2_msgs.msg
from mavros_msgs.msg import OverrideRCIn, RCIn


ros_mode = True

def add_points(P1, P2):
    return Point(P1.x+P2.x, P1.y+P2.y, P1.z+P2.z)

def subtract_points(P1, P2):
    return Point(P1.x-P2.x, P1.y-P2.y, P1.z-P2.z)

def rc_callback(msg):
    if msg.channels[4] < 1621:
        global ros_mode
        if ros_mode:
            rospy.loginfo("ending_auto with value {}".format(msg.channels[0]))
        ros_mode = False
    else:
        ros_mode = True

def clamp_rc(value):
    if value == 0:
        return 0
    elif value < 1100:
        return 1100
    elif value > 1900:
        return 1900
    return value


current_pose = PoseStamped()
target = Point()
def main():
    rospy.init_node('position_hold')
    target_offset = Point()
    target_offset.x = 0.5
    tfBuffer = tf2_ros.Buffer(rospy.Duration(10))
    listener = tf2_ros.TransformListener(tfBuffer)
    last_time = rospy.Time.now()

    def update_current_pose(msg):
        global current_pose
        current_pose = msg


    rc_pub = rospy.Publisher("mavros/rc/override", OverrideRCIn, queue_size=10)
    rc_sub = rospy.Subscriber("mavros/rc/in", RCIn, rc_callback)
    pose_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, update_current_pose)


    def ar_callback(msgs):
        for msg in msgs.markers:
            if msg.id == 0:
                global last_time
                last_time = msg.header.stamp
                global target
                target = msg.pose.pose.position


    ar_sub = rospy.Subscriber("ar_pose_marker", AlvarMarkers, ar_callback)



    rate = rospy.Rate(10)
    trans = None
    while not rospy.is_shutdown():
        rc = OverrideRCIn()
        try:
            trans = tfBuffer.lookup_transform_full(target_frame='base_link', target_time=rospy.Time.now(), source_frame='ar_marker_0', source_time=rospy.Time.now(),   fixed_frame='map', timeout=rospy.Duration(1))
            rospy.loginfo(trans)
        except Exception as e:
            if trans is None:
                continue
            pass

        pos_error = subtract_points(trans.transform.translation, target_offset)
        rc.channels[1] = 1500 + int(500 * pos_error.x)
        # rc.channels[1] = 1450
        angle = math.atan2(pos_error.y, pos_error.x)
        if rc.channels[1] < 1500:
            rc.channels[3] = 1500 + int(1000 * angle)
        else:
            rc.channels[3] = 1500 - int(1000 * angle)
        rospy.loginfo("{}cm: {}rad".format(trans.transform.translation.x * 100, angle))
        rc.channels = [clamp_rc(val) for val in rc.channels]
        if ros_mode:
            rc_pub.publish(rc)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
