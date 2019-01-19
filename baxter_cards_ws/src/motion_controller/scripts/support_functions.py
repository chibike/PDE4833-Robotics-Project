#!/usr/bin/env python

from __future__ import division

import tf
import cv2
import copy
import math
import rospy
import tf2_ros
import numpy as np
from vector_illustration_processing import pi_point, pi_line

from geometry_msgs.msg import Transform, TransformStamped
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def to_degrees(array):
    new_array = copy.deepcopy(array)

    for i in range(len(array)):
        new_array[i] = math.degrees(array[i])
    
    return new_array

def to_pi_point(transform):
    return pi_point.Point(transform.translation.x, transform.translation.y, transform.translation.z)

def convert_transforms_to_point_array(transforms):
    array = []
    
    for transform in transforms:
        array.append(to_pi_point(transform))

    return array

def convert_transforms_to_point(transforms):
    point_array = convert_transforms_to_point_array(transforms)

    total = 0
    for point in point_array:
        total = point.add(total)

    return total.div(len(point_array))