#!/usr/bin/env python
import math

import numpy as np

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose


class PathGenerator:

    def __init__(self):
        pass

    def generate_path_now(self, current_pose, target_pose):
        pass

class PublisherClass:

    def __init__(self, publish_path, class_type):
        self._pub = rospy.Publisher(publish_path, class_type, queue_size=10)

    def publish(self):
        new_value = self.publish_value()
        if isinstance(new_value, self._pub.data_class):
            self._pub.publish(new_value)

    def publish_value(self): raise NotImplementedError("Must specify what to publish")

class StraightLinePath(PathGenerator, PublisherClass):

    def __init__(self, pub_string, length=1.0):
        """length is in m"""
        PathGenerator.__init__(self)
        PublisherClass.__init__(self, pub_string, Path)
        self.path = Path()
        self.publish_value = lambda:self.path
        self.path_length = length

    def generate_path_now(self, current_pose, target_pose):
        self.path = Path()
        self.path.header.frame_id = "dock_frame"
        for x in np.arange(self.path_length, target_pose.pose.position.x, -0.1):
            next_loc = PoseStamped()
            next_loc.header.frame_id = "dock_frame"
            next_loc.pose.position.x = x
            next_loc.pose.orientation.z = 1
            self.path.poses.append(next_loc)


class QuadraticPath(PathGenerator, PublisherClass):

    def __init__(self, pub_string, a=1.0):
        """length is in m"""
        PathGenerator.__init__(self)
        PublisherClass.__init__(self, pub_string, Path)
        self.path = Path()
        self.publish_value = lambda:self.path

    def generate_path_now(self, current_pose, target_pose):
        self.path = Path()
        self.path.header.frame_id = "dock_frame"
        a = current_pose.pose.position.y/current_pose.pose.position.x**2
        for x in np.arange(current_pose.pose.position.x, target_pose.pose.position.x, -0.1):
            next_loc = PoseStamped()
            next_loc.header.frame_id = "dock_frame"
            next_loc.pose.position.x = x
            next_loc.pose.position.y = a*x**2
            theta = math.atan2(a*x*2, 1)+np.pi
            next_loc.pose.orientation.z = math.sin(theta/2)
            next_loc.pose.orientation.w = math.cos(theta/2)
            self.path.poses.append(next_loc)


class GradientPath(PathGenerator, PublisherClass):

    def __init__(self, pub_string, a=1.0):
        """length is in m"""
        PathGenerator.__init__(self)
        PublisherClass.__init__(self, pub_string, Path)
        self.path = Path()
        self.publish_value = lambda:self.path
        self.a = a

    def gradient(self, x, y):
        return -2*np.array([x, (1+self.a)*y])

    def generate_path_now(self, current_pose, target_pose):
        self.path = Path()
        self.path.header.frame_id = "dock_frame"
        position = current_pose.pose.position
        while position.x > 0.05:
            next_loc = PoseStamped()
            next_loc.header.frame_id = "dock_frame"

            gradient = self.gradient(position.x, position.y)
            rospy.loginfo("{}:{}".format(gradient,np.linalg.norm(gradient)))
            gradient /= 10*np.linalg.norm(gradient)
            position.x += gradient[0]
            position.y += gradient[1]

            theta = math.atan2(gradient[1], gradient[0])

            next_loc.pose.position.x = position.x
            next_loc.pose.position.y = position.y
            next_loc.pose.orientation.z = np.sin(theta/2)
            next_loc.pose.orientation.w = np.cos(theta/2)
            self.path.poses.append(next_loc)

def main():
    rospy.init_node('path_generator')
    generator = GradientPath("/path")
    generator.a = 8
    current = PoseStamped()
    current.pose.position.x = 2.0
    current.pose.position.y = 2.0
    generator.generate_path_now(current, PoseStamped())
    generator.publish()

    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        generator.publish()
        rate.sleep()


if __name__ == '__main__':
    main()


