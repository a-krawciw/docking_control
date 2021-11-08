import numpy as np
import tf.transformations
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


def min_dist(path, pose):
    if not path.header.frame_id == pose.header.frame_id:
        raise ValueError("path and pose must be in the same frame")
    x_current = pose.pose.position.x
    y_current = pose.pose.position.y
    path_dist = [np.sqrt((path_i.pose.position.x-x_current)**2+(path_i.pose.position.y-y_current)**2) for path_i in path.poses]
    return np.nanmin(path_dist), np.argmin(path_dist)


def extract_yaw(q):
    return tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

def theta_diff(t1, t2):
    t1 = np.unwrap(t1)
    t2 = np.unwrap(t2)
    return np.unwrap(t1 - t2)
