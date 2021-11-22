#!/usr/bin/env python

import numpy as np
import rospy
from sensor_msgs.msg import Imu
from contoller_utils import extract_yaw, extract_pitch, extract_roll


def extract_variance(msgs, extractor):
    return np.var([extractor(msg) for msg in msgs])

def main():
    rospy.init_node("std_dev")
    msgs = []

    def add_measurement(msg):
        msgs.append(msg)

    imu_sub = rospy.Subscriber("imu/data", Imu, add_measurement)

    while not rospy.is_shutdown() and len(msgs) < 1000:
        rospy.sleep(1)
    rospy.loginfo("Variance for angular_vel.x:{}".format(extract_variance(msgs, lambda x: x.angular_velocity.x)))
    rospy.loginfo("Variance for angular_vel.y:{}".format(extract_variance(msgs, lambda x: x.angular_velocity.y)))
    rospy.loginfo("Variance for angular_vel.z:{}".format(extract_variance(msgs, lambda x: x.angular_velocity.z)))

    rospy.loginfo("Variance for lin_accel.x:{}".format(extract_variance(msgs, lambda x: x.linear_acceleration.x)))
    rospy.loginfo("Variance for lin_accel.y:{}".format(extract_variance(msgs, lambda x: x.linear_acceleration.y)))
    rospy.loginfo("Variance for lin_accel.z:{}".format(extract_variance(msgs, lambda x: x.linear_acceleration.z)))

    rospy.loginfo("Variance for orientation.x:{}".format(extract_variance(msgs, lambda x: extract_roll(x.orientation))))
    rospy.loginfo("Variance for orientation.y:{}".format(extract_variance(msgs, lambda x: extract_pitch(x.orientation))))
    rospy.loginfo("Variance for orientation.z:{}".format(extract_variance(msgs, lambda x: extract_yaw(x.orientation))))

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
