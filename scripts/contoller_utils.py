import numpy as np
import tf.transformations
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


def min_dist(path, pose):
    if not path.header.frame_id == pose.header.frame_id:
        raise ValueError("path and pose must be in the same frame frames are {} and {}".format(path.header.frame_id, pose.header.frame_id))
    x_current = pose.pose.position.x
    y_current = pose.pose.position.y
    path_dist = [np.sqrt((path_i.pose.position.x-x_current)**2+(path_i.pose.position.y-y_current)**2) for path_i in path.poses]
    return np.nanmin(path_dist), \
           np.argmin(path_dist)

def extract_roll(q):
    return tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[0]

def extract_pitch(q):
    return tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[1]

def extract_yaw(q):
    return tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

def theta_diff(t1, t2):
    t1 = unwrap2Pi(t1)
    t2 = unwrap2Pi(t2)
    return t1 - t2

def unwrap2Pi(v):
    v = np.unwrap([v])
    return v if v > 0 else v + 2*np.pi
