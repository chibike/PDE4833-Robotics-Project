#!/usr/bin/env python

from __future__ import division

import tf
import cv2
import copy
import math
import rospy
import thread
import random
import tf2_ros
import numpy as np
import cv2.aruco as aruco
from std_msgs.msg import Bool
from geometry_msgs.msg import Transform, TransformStamped
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import card_bin
import support_functions
from vector_illustration_processing import pi_point, pi_line

class CardSearchRoutine(object):
    def __init__(self, motion_control_object, media_access_object, card_bin_solver):
        self.card_bin_solver = card_bin_solver
        self.media_access_object = media_access_object
        self.motion_control_object = motion_control_object

        self.cancel_triggered = False
        self.status_change_publisher = None
        self.should_search_for_aruco_markers = False

        # highest verbosity level - print all logs
        self.verbosity_level = 0

        self.cards_bin_found = False
        self.marker_transforms = {0:[], 1:[], 2:[], 3:[]}
        self.processed_marker_positions = {0:None, 1:None, 2:None, 3:None}
        self.clear_marker_data()

        self.tag_names = ["top_left", "bottom_left", "top_right", "bottom_right"]
        
        self.processed_image = None
        self.camera_scan_poses = [
            [[0.6407, 0.4677, 0.0080], [-0.0401, 0.7469, -0.1941, 0.6345]],
            [[0.7037, 0.2513, -0.0275], [-0.0019, 0.8185, -0.0292, 0.5736]],
            [[0.7852, 0.1002, -0.0422], [-0.1717, 0.7094, 0.07973, 0.6788]],
            ]
        #self.camera_scan_poses = [[[0.785261162784, 0.100295329394, -0.0422338188207], [-0.171711149326, 0.709420460354, 0.0797348193394, 0.67888161723]], [[0.804113134994, 0.283772401894, 0.00341792150348], [-0.074454567414, 0.787508783552, -0.00902939681809, 0.611722897407]], [[0.733510893977, 0.379406611302, -0.0338807017615], [0.221085874774, 0.670487861849, -0.200286352841, 0.679302907364]], [[0.852484690015, 0.335287599352, -0.0805636665798], [0.0320873010964, 0.70712954829, -0.00807705008832, 0.706309399842]]]
        self.card_approach_home_pose = [[0.6951, 0.2911, 0.0508], [0.7052, 0.7055, 0.0696, -0.0015]]
        
    def initialize(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
    
    def terminate(self):
        pass
    
    def clear_marker_data(self):
        self.cards_bin_found = False
        self.marker_transforms = {0:[], 1:[], 2:[], 3:[]}
        self.processed_marker_positions = {0:None, 1:None, 2:None, 3:None}

        self.publish_status()
    
    def start_image_viewer(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.media_access_object.update_display(self.processed_image)
            rate.sleep()
    
    def process_marker_transforms(self):
        for tag in self.marker_transforms.keys():
            if not self.is_marker_transform_available(tag):
                rospy.loginfo("[INFO] skipping processing of marker position with tag: `{}`".format(tag))
                continue
            
            self.processed_marker_positions[tag] = support_functions.convert_transforms_to_point(self.marker_transforms[tag])
        
        return True    
    
    def is_marker_transform_available(self, tag):
        if not isinstance(self.marker_transforms[tag], list):
            return False
        
        return len( self.marker_transforms[tag] ) > 0

    def found_left(self):
        return self.is_marker_transform_available(0) or self.is_marker_transform_available(1)
        
    def found_right(self):
        return self.is_marker_transform_available(2) or self.is_marker_transform_available(3)
    
    def found_top(self):
        return self.is_marker_transform_available(0) or self.is_marker_transform_available(2)
    
    def found_bottom(self):
        return self.is_marker_transform_available(1) or self.is_marker_transform_available(3)
    
    def get_average_marker_point(self, tag_a, tag_b):
        p1 = self.processed_marker_positions[tag_a]
        p2 = self.processed_marker_positions[tag_b]

        p = None
        if isinstance(p1, pi_point.Point) and isinstance(p2, pi_point.Point):
            p = (p1 + p2).div(2.0)
        elif isinstance(p1, pi_point.Point):
            p = p1
        elif isinstance(p2, pi_point.Point):
            p = p2
        else:
            raise ValueError("logic error occurred while processing p, p1, and p2")
        return p
    
    def get_left_marker_point(self):
        return self.get_average_marker_point(2,3)
    
    def get_right_marker_point(self):
        return self.get_average_marker_point(0, 1)
    
    def get_top_marker_point(self):
        return self.get_average_marker_point(0,2)
    
    def get_bottom_marker_point(self):
        return self.get_average_marker_point(1,3)
    
    def start_search(self):
        rospy.loginfo("[INFO] may the search begin")

        # clear previously found data
        self.clear_marker_data()

        # scan for new markers
        n_markers_found = self.scan_location(self.camera_scan_poses)
        
        success = False
        if (n_markers_found >= 3) or (self.found_left() and self.found_right()):
            rospy.loginfo("[INFO] successfully located your cards")
            success = True
        else:
            rospy.loginfo("[INFO] could not find your cards, please try again")
            success = False

        if not success:
            return False
        
        if not self.process_marker_transforms():
            rospy.loginfo("[INFO] could not process your card position, please try again")
            return False

        z_position = -0.1745636
        left_right_line = pi_line.Line(self.get_left_marker_point(), self.get_right_marker_point())
        left_right_center_point = left_right_line.get_midpoint()

        if self.found_top() and self.found_bottom():
            # rospy.loginfo("[INFO] updated your cards z position")
            calculated_z_position = pi_line.Line(self.get_top_marker_point(), self.get_bottom_marker_point()).get_midpoint().z
            rospy.loginfo("[INFO] calculated z position {}".format(calculated_z_position))

        card_bin_center_transform = Transform()
        card_bin_center_transform.translation.x = left_right_center_point.x
        card_bin_center_transform.translation.y = left_right_center_point.y - 0.3
        card_bin_center_transform.translation.z = z_position

        # rotation = quaternion_from_euler(math.pi / 2.0, 0, marker_xy_line.get_angle())
        rospy.loginfo("[INFO] calculated line angle {} rad, {} deg".format(left_right_line.get_angle(), math.degrees(left_right_line.get_angle())))
        rotation = quaternion_from_euler(0, 0, 0)

        card_bin_center_transform.rotation.x = rotation[0]
        card_bin_center_transform.rotation.y = rotation[1]
        card_bin_center_transform.rotation.z = rotation[2]
        card_bin_center_transform.rotation.w = rotation[3]

        self.card_bin_solver.update_broadcast_transform(card_bin_center_transform)

        rospy.loginfo("[INFO] processed positions: {}".format(self.processed_marker_positions))

        left_p = self.get_left_marker_point()
        right_p = self.get_right_marker_point()
        def __get_f(p, n=4):
            return round(p.x, n), round(p.y, n), round(p.z, n)
        rospy.loginfo("[INFO] left: {}, right: {}".format(__get_f(left_p), __get_f(right_p)))

        rospy.loginfo("[INFO] moving to card approach pose")
        self.goto_card_approach_pose(); rospy.sleep(3)

        # self.goto_point(left_right_center_point); rospy.sleep(10)
        # self.goto_point(self.get_left_marker_point()); rospy.sleep(10)
        # self.goto_point(self.get_right_marker_point()); rospy.sleep(10)

        # testing moving to camera pose
        self.goto_picture_pose(); rospy.sleep(5)

        self.cards_bin_found = True
        self.publish_status()

        # testing pickup position 1
        # self.goto_card_approach_pose()
        # self.pickup_card(1); rospy.sleep(15)

        # self.goto_card_approach_pose()
        # self.pickup_card(2); rospy.sleep(15)

        # self.goto_card_approach_pose()
        # self.pickup_card(3); rospy.sleep(15)

        # self.goto_card_approach_pose()
        # self.pickup_card(4)

    def cancel_search(self):
        rospy.loginfo("[INFO] terminating search")

        self.cancel_triggered = True
        self.stop_search_for_markers()

        rospy.sleep(3)
        self.motion_control_object.open_gripper()
        self.motion_control_object.move_to_neutral(which_arm="right")
        self.motion_control_object.move_to_home("left")

        self.cancel_triggered = False

    def get_search_status(self):
        return self.cards_bin_found
    
    def scan_location(self, camera_scan_poses, delay=4):
        def __scan_location(pose_as_array):
            self.goto_card_approach_pose(); rospy.sleep(1)

            pose = Pose()
            pose.position.x = pose_as_array[0][0]
            pose.position.y = pose_as_array[0][1]
            pose.position.z = pose_as_array[0][2]

            pose.orientation.x = pose_as_array[1][0]
            pose.orientation.y = pose_as_array[1][1]
            pose.orientation.z = pose_as_array[1][2]
            pose.orientation.w = pose_as_array[1][3]

            self.stop_search_for_markers(); rospy.sleep(1)
            rospy.loginfo("[INFO] scanning position {}".format(pose))

            self.motion_control_object.move_to("left", pose, 0.07, True); rospy.sleep(2)
            self.start_search_for_markers()

            rospy.sleep(delay)

            self.stop_search_for_markers();
            rospy.loginfo("[INFO] finished scanning")

        for camera_scan_pose in camera_scan_poses:
            __scan_location(camera_scan_pose)

            if self.cancel_triggered:
                break

        howmany = self.count_markers_found()
        rospy.loginfo("[INFO] found {} markers".format(howmany))

        return howmany
    
    def test_scan_location(self, camera_pose):
        rospy.loginfo("[INFO] testing scan location\n{}\n".format(camera_pose))

        self.clear_marker_data()
        howmany = self.scan_location([camera_pose])
        self.process_marker_transforms()

        rospy.loginfo("[INFO] found {} markers".format(howmany))
        rospy.loginfo("[INFO] processed marker positions:\n{}\n".format(self.processed_marker_positions))

        return howmany
    
    def goto_card_approach_pose(self):
        pose = Pose()
        pose.position.x = self.card_approach_home_pose[0][0]
        pose.position.y = self.card_approach_home_pose[0][1]
        pose.position.z = self.card_approach_home_pose[0][2]

        pose.orientation.x = self.card_approach_home_pose[1][0]
        pose.orientation.y = self.card_approach_home_pose[1][1]
        pose.orientation.z = self.card_approach_home_pose[1][2]
        pose.orientation.w = self.card_approach_home_pose[1][3]

        self.motion_control_object.move_to("left", pose, 0.07, True); rospy.sleep(1)
    
    def goto_picture_pose(self):
        q = [-0.0106144682496, 0.740517609283, -0.0168285256464, 0.671742364401]

        camera_transform = self.card_bin_solver.get_camera_transform()
        camera_pose = card_bin.transform_to_pose(camera_transform)

        camera_pose.orientation.x = q[0]
        camera_pose.orientation.y = q[1]
        camera_pose.orientation.z = q[2]
        camera_pose.orientation.w = q[3]

        if camera_pose is None:
            rospy.loginfo("[INFO] could not go to camera pose, please try again")
            return False
        
        return self.motion_control_object.move_to("left", camera_pose, 0.04, True)
    
    def goto_point(self, pi_point):
        q = [-0.0238422831645, 0.691517335143, -0.0495474119488, 0.72026410066]
        z_position = 0.1

        pose = Pose()
        pose.position.x = pi_point.x
        pose.position.y = pi_point.y
        pose.position.z = z_position # pi_point.z

        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        rospy.loginfo("[INFO] going to pose {}".format(pose))
        return self.motion_control_object.move_to("left", pose, 0.04, True)
    
    def pickup_card(self, card_id, hover_distance=0.1):
        q = [0.707697538914, 0.703539453336, 0.0636996548328, 0.0117807040839]

        pickup_transform = self.card_bin_solver.get_pickup_transform(card_id)
        pickup_pose = card_bin.transform_to_pose(pickup_transform)

        pickup_pose.position.x -= 0.05

        pickup_pose.orientation.x = q[0]
        pickup_pose.orientation.y = q[1]
        pickup_pose.orientation.z = q[2]
        pickup_pose.orientation.w = q[3]

        rospy.loginfo("[INFO] pickup_pose {} is {}".format(card_id, pickup_pose))

        approach_pose = copy.deepcopy(pickup_pose)
        approach_pose_2 = copy.deepcopy(pickup_pose)

        if pickup_pose is None:
            rospy.loginfo("[INFO] cannot pickup card from an invalid position")
            return False
        
        approach_pose.position.z += hover_distance
        approach_pose_2.position.z += hover_distance / 3.0

        rospy.loginfo("[INFO] moving to card approach pose")
        self.goto_card_approach_pose()

        rospy.loginfo("[INFO] approaching object")
        self.motion_control_object.move_to("left", approach_pose, 0.05, True)
        self.motion_control_object.open_gripper(block=True)

        self.motion_control_object.move_to("left", approach_pose_2, 0.02, True)

        pickup_pose.position.z -= hover_distance / 3.0
        self.motion_control_object.move_to("left", pickup_pose, 0.03, True)
        self.motion_control_object.close_gripper(block=True)

        self.motion_control_object.move_to("left", approach_pose_2, 0.02, True)

        # rospy.loginfo("[INFO] moving to object")
        # self.motion_control_object.move_linear("left", approach_pose, pickup_pose, 0.3, 0.04, True)
        # self.motion_control_object.close_gripper(block=True)

        rospy.loginfo("[INFO] moving to approach pose")
        self.motion_control_object.move_to("left", approach_pose, 0.03, True)

        return True
    
    def count_markers_found(self):
        valid_tags = [tag for tag in self.marker_transforms.keys() if self.is_marker_transform_available(tag)]
        return len(valid_tags)
    
    def get_transform_from_vectors(self, tvec, rvec):
        transform = Transform()
        transform.translation.x = tvec[0]
        transform.translation.y = tvec[1]
        transform.translation.z = tvec[2]

        orientation = quaternion_from_euler(rvec[0], rvec[1], rvec[2])

        transform.rotation.x = orientation[0]
        transform.rotation.y = orientation[1]
        transform.rotation.z = orientation[2]
        transform.rotation.w = orientation[3]

        return transform
    
    def publish_status(self):
        if self.status_change_publisher is not None:
            msg = Bool()
            msg.data = self.cards_bin_found
            self.status_change_publisher.publish(msg)
    
    def start_search_for_markers(self, fps=5):
        self.should_search_for_aruco_markers = True

        def flip_from_image_coord(vector):
            new_vector = copy.deepcopy(vector)
            new_vector[0,0] = copy.deepcopy(vector[0,2])
            new_vector[0,1] = copy.deepcopy(-1.0 * vector[0,0])
            new_vector[0,2] = copy.deepcopy(+1.0 * vector[0,1])

            return new_vector

        def __search_func():
            rospy.loginfo("[INFO] searching for visible aruco markers")

            # initialize variables
            camera_matrix = np.asmatrix([[405.055352185, 0.0, 641.288633522],[0.0, 405.055352185, 394.79338426],[0.0, 0.0, 1.0]])
            dist_coeffs = (0.0184666546383, -0.0531712194253, 0.00187038429393, -0.000381197982797, 0.0132522427926)
            aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
            parameters =  aruco.DetectorParameters_create()
            marker_size = 0.025

            # set loop rate
            rate = rospy.Rate(fps)
            while self.should_search_for_aruco_markers:

                # get image and extract aruco markers
                img = self.media_access_object.left_hand_cam_image
                img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                corners, tags, rejectedImgPoints = aruco.detectMarkers(img, aruco_dict, parameters=parameters)

                # validate tags
                if tags is None:
                    continue
                elif len(tags) <= 0:
                    continue

                # draw markers
                aruco.drawDetectedMarkers(img, corners, tags)

                # estimate markers positions
                rvecs,tvecs,_obj_points = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, dist_coeffs)

                # get left camera's position
                left_hand_camera_position_result = self.motion_control_object.get_left_hand_camera_transform()
                if not left_hand_camera_position_result["status"]:
                    rospy.loginfo("[INFO] could not get the left hand camera's position")
                    continue
                
                # loop through all tags found
                for i in range(len(tags)):
                    tag = tags[i][0]

                    # reject undefined tags
                    if not (tag in self.marker_transforms.keys()):
                        continue

                    tvec = tvecs[i]
                    rvec = rvecs[i]

                    # convert vectors from image coords to world coords
                    n_tvec = flip_from_image_coord(tvec)

                    # draw coord axis on image and save it to be displayed { @ self.start_image_viewer }
                    self.processed_image = aruco.drawAxis( img, camera_matrix, dist_coeffs, rvec, tvec, marker_size )

                    # calculate transform from robot's base
                    camera_transform = left_hand_camera_position_result["transform"]
                    marker_transform = self.get_transform_from_vectors(n_tvec.ravel(), rvec.ravel())
                    combined_transform = card_bin.combine_transforms(camera_transform, marker_transform)

                    # stored transform to be processed later
                    self.marker_transforms[tag].append(combined_transform)
                    
                    if self.verbosity_level >= 0.5:
                        # print log
                        rospy.loginfo("[INFO] found tag '{}'".format(self.tag_names[tag]))
                        rospy.loginfo("[INFO] with tvec: {} and rvec: {}".format(tvec, support_functions.to_degrees(rvec.ravel())))
                        rospy.loginfo("[INFO] marker transform\n{}".format(marker_transform))
                        rospy.loginfo("[INFO] camera transform\n{}".format(camera_transform))
                        rospy.loginfo("[INFO] combined transform\n{}".format(combined_transform))

                rate.sleep()
            rospy.loginfo("[INFO] terminated search for visible aruco markers")
        
        thread.start_new_thread(__search_func, ())
    
    def stop_search_for_markers(self):
        self.should_search_for_aruco_markers = False
    
    def broadcast_stamped_transform(self, transform_stamped):
        self.tf_broadcaster.sendTransform(transform_stamped)