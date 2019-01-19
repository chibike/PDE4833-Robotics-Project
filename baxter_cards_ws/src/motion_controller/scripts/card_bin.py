#!/usr/bin/env python

from __future__ import division

import tf
import math
import rospy
import thread
import tf2_ros
import numpy as np
from geometry_msgs.msg import Transform, TransformStamped
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler, translation_matrix, quaternion_matrix, translation_from_matrix, quaternion_from_matrix

def combine_transforms(transform_1, transform_2):
    matrix_1 = np.dot( translation_matrix([transform_1.translation.x, transform_1.translation.y, transform_1.translation.z]), quaternion_matrix([transform_1.rotation.x, transform_1.rotation.y, transform_1.rotation.z, transform_1.rotation.w]) )
    matrix_2 = np.dot( translation_matrix([transform_2.translation.x, transform_2.translation.y, transform_2.translation.z]), quaternion_matrix([transform_2.rotation.x, transform_2.rotation.y, transform_2.rotation.z, transform_2.rotation.w]) )

    matrix = np.dot(matrix_1, matrix_2)
    
    transform = Transform()
    r = quaternion_from_matrix(matrix)
    transform.rotation.x = r[0]
    transform.rotation.y = r[1]
    transform.rotation.z = r[2]
    transform.rotation.w = r[3]

    t = translation_from_matrix(matrix)
    transform.translation.x = transform_1.translation.x + transform_2.translation.x  # t[0] # 
    transform.translation.y = transform_1.translation.y + transform_2.translation.y  # t[1] # 
    transform.translation.z = transform_1.translation.z + transform_2.translation.z  # t[2] # 

    return transform

def transform_to_pose(transform):
    pose = Pose()
    pose.position.x = transform.translation.x
    pose.position.y = transform.translation.y
    pose.position.z = transform.translation.z
    
    pose.orientation.x = transform.rotation.x
    pose.orientation.y = transform.rotation.y
    pose.orientation.z = transform.rotation.z
    pose.orientation.w = transform.rotation.w

    return pose

def pose_to_transform(pose):
    transform = Transform()
    transform.translation.x = pose.position.x
    transform.translation.y = pose.position.y
    transform.translation.z = pose.position.z

    transform.rotation.x = pose.orientation.x
    transform.rotation.y = pose.orientation.y
    transform.rotation.z = pose.orientation.z
    transform.rotation.w = pose.orientation.w

    return transform

def apply_transform_to_pose(self, transform, pose):
    pose_as_transform = pose_to_transform(pose)
    transform = combine_transforms(pose_as_transform, transform)
    
    return transform_to_pose(transform)


class CardBinPositionSolver(object):
    def __init__(self, reference_frame="base", card_bin_frame="card_bin_object"):
        self.reference_frame = reference_frame
        self.card_bin_frame = card_bin_frame

        self.tf_transformer = None

        self.broadcast_rate = 10
        self.current_transform = None
        self.should_broadcast_transform = False

    def initialize(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_transformer = tf.TransformerROS()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.center_transform = Transform()
        self.center_transform.translation.x = 0
        self.center_transform.translation.y = 0
        self.center_transform.translation.z = 0

        q = quaternion_from_euler(0,0,0)

        self.center_transform.rotation.x = q[0]
        self.center_transform.rotation.y = q[1]
        self.center_transform.rotation.z = q[2]
        self.center_transform.rotation.w = q[3]

        self.update_broadcast_transform(self.center_transform)
        self.start_broadcast()
    
    def terminate(self):
        rospy.loginfo("[INFO] terminating card bin position solver")

        self.stop_broadcast()
    
    def update(self, tag_name, transform_stamped):

        pose_stamped = self.transform_stamped_to_pose_stamped(transform_stamped)

        tag_names = [
            "top_right_tag",
            "top_left_tag",
            "bottom_left_tag",
            "bottom_right_tag",
        ]

        tag_name = "{}_tag".format(tag_name)
        if not tag_name in tag_names:
            rospy.loginfo("[INFO] could not find tag name: '{}' in the list of tag names".format(tag_name))
            return False

        msg = "[BEFORE] received tag: `{}` with pose: {}".format(tag_name, pose_stamped)
        rospy.loginfo("[INFO] {}".format(msg))
        
        pose_stamped = self.transform_stamped_pose(self.reference_frame, pose_stamped)

        msg = "[AFTER] received tag: `{}` with pose: {}".format(tag_name, pose_stamped)
        rospy.loginfo("[INFO] {}".format(msg))
        
        # TODO: update center bin position using the given pose
        pass

    def get_pickup_transform(self, slot_id):
        slot_frame_ids = [
            "pickup_slot_1",
            "pickup_slot_2",
            "pickup_slot_3",
            "pickup_slot_4"
        ]
        
        slot_frame_id = "pickup_slot_{}".format(slot_id)
        if not slot_frame_id in slot_frame_ids:
            rospy.loginfo("[INFO] could not find slot id '{}' in the list of slot ids".format(slot_id))
            return None

        result = self.get_transform_from(self.card_bin_frame, slot_frame_id)

        if not result["status"]:
            rospy.loginfo("[INFO] could not calculate the transform between the base and {}".format(slot_frame_id))
            return None

        return combine_transforms(self.current_transform, result["transform"])
    
    def get_camera_transform(self):
        result = self.get_transform_from(self.card_bin_frame, "camera_picture_pose")

        if not result["status"]:
            rospy.loginfo("[INFO] could not calculate the transform between the base and the camera_picture_pose")
            return None

        return combine_transforms(self.current_transform, result["transform"])
    
    def transform_stamped_to_pose_stamped(self, transform_stamped):
        pose_stamped = PoseStamped()
        pose_stamped.header = transform_stamped.header
        # pose_stamped.header.frame_id = transform_stamped.child_frame_id
        pose_stamped.pose.position = transform_stamped.transform.translation
        pose_stamped.pose.orientation = transform_stamped.transform.rotation

        return pose_stamped
    
    def transform_stamped_pose(self, new_frame, pose_stamped):
        # return self.tf_listener.transformPose(new_frame, pose_stamped)
        return self.tf_buffer.transformPose(new_frame, pose_stamped)
        
    def get_transform_from(self, origin_frame_id, frame_id):
        transform_stamped = None
        result = {
            "status" : False,
            "msg" : "Did not run",
            "position":None,
            "rotation":None,
            "transform":None
        }

        try:
            transform_stamped = self.tf_buffer.lookup_transform(origin_frame_id, frame_id, rospy.Time())
            # transform_stamped = self.tf_listener.lookup_transform(origin_frame_id, frame_id, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            result["msg"] = "could not lookup the transform between {} and {}".format(origin_frame_id, frame_id)
            rospy.loginfo("[ERROR] {}".format(result["msg"]))

        if transform_stamped is None:
            return False
        
        rospy.loginfo("[INFO] transform_stamped {}".format(transform_stamped))

        result["transform"] = transform_stamped.transform
        result["position"] = transform_stamped.transform.translation.x, transform_stamped.transform.translation.y, transform_stamped.transform.translation.z
        result["rotation"] = euler_from_quaternion((transform_stamped.transform.rotation.x, transform_stamped.transform.rotation.y, transform_stamped.transform.rotation.z, transform_stamped.transform.rotation.w))
        result["status"] = True
        result["msg"] = "successful"

        return result
    
    def start_broadcast(self):
        self.should_broadcast_transform = True

        def func():
            rate = rospy.Rate(self.broadcast_rate)

            rospy.loginfo("[INFO] started broadcasting card_bin tf")

            while self.should_broadcast_transform and (not rospy.is_shutdown()):
                self.broadcast_transform(self.current_transform)
                rate.sleep()
            
            rospy.loginfo("[INFO] terminated broadcast card_bin tf")
        
        thread.start_new_thread(func, ())
    
    def stop_broadcast(self):
        self.should_broadcast_transform = False
        rospy.sleep(3)

    
    def update_broadcast_transform(self, transform):
        self.current_transform = transform
    
    def broadcast_transform(self, transform):
        self.stamped_transform = TransformStamped()
        self.stamped_transform.header.stamp = rospy.Time.now()
        self.stamped_transform.header.frame_id = self.reference_frame
        self.stamped_transform.child_frame_id = self.card_bin_frame

        self.stamped_transform.transform = transform
        self.tf_broadcaster.sendTransform(self.stamped_transform)