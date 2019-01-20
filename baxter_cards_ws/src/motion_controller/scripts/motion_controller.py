#!/usr/bin/env python

from __future__ import division

import tf
import sys
import math
import rospy
import struct
import tf2_ros
import actionlib
import baxter_interface
from copy import deepcopy
from geometry_msgs.msg import TransformStamped
from tf.transformations import euler_from_quaternion

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest

from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

from vector_illustration_processing import pi_point, pi_line

class JointTrajectoryBuilder(object):
    def __init__(self, limb, joint_names):
        ns = "robot/limb/{limb}".format(limb=limb)
        self._client = actionlib.SimpleActionClient(
            "{ns}/follow_joint_trajectory".format(ns=ns),
            FollowJointTrajectoryAction
            )
        
        self.joint_names = joint_names
        self.points = []
        
        self._goal = FollowJointTrajectoryGoal()
        self._goal_time_tolerance = rospy.Time(0.1)
        self._goal.goal_time_tolerance = self._goal_time_tolerance

        if not self._client.wait_for_server(timeout=rospy.Duration(10.0)):
            raise RuntimeError("Could not connect with action server")
        
        self.clear(limb)
    
    def clear(self, limb):
        self._goal = FollowJointTrajectoryGoal()
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        self._goal.trajectory.joint_names = self.joint_names

        self.points = []
        self._goal.trajectory.points = self.points
    
    def result(self):
        return self._client.get_result()
    
    def wait(self, timeout=15.0):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))
    
    def stop(self):
        self._client.cancel_goal()
    
    def start(self):
        self._goal.trajectory.header.stamp = rospy.Time.now()
        self._client.send_goal(self._goal)
    
    def add_joint_values(self, joint_values, time_from_start):
        point = JointTrajectoryPoint()
        point.positions = [joint_values[joint_name] for joint_name in self.joint_names]
        point.time_from_start = rospy.Duration(time_from_start)

        self.points.append(point)
        self._goal.trajectory.points = self.points

class MotionController(object):
    def __init__(self):
        self.left_limb = None
        self.right_limb = None
        self.left_gripper = None

        self.home_joint_angles = {'right': {'right_s0': 0.0782330201821561, 'right_s1': -0.9978545025194616, 'right_w0': -0.6695826139119831, 'right_w1': 1.0316020798529408, 'right_w2': 0.49854375606275947, 'right_e0': 1.1888351106111956, 'right_e1': 1.940102201478077}, 'left': {'left_w0': 0.6718835850938112, 'left_w1': 1.031985575049912, 'left_w2': -0.4989272512597308, 'left_e0': -1.1876846250202815, 'left_e1': 1.9381847254932203, 'left_s0': -0.08130098175792692, 'left_s1': -0.9978545025194616}}
        
        left_home_pose = self.__to_pose({'position': Point(x=0.579855212418682, y=0.18100081859198275, z=0.10624652227076539), 'orientation': Quaternion(x=0.14215456279682886, y=0.9893808250576214, z=0.011527863145743753, w=0.028013776175947764)})
        right_home_pose = self.__to_pose({'position': Point(x=0.57398936577748, y=-0.1765983136335029, z=0.2545465393848954), 'orientation': Quaternion(x=-0.11599854624179622, y=0.9929851932636404, z=-0.010979742760207552, w=0.020104439258041465)})
        self.home_pose = {'right':right_home_pose, 'left':left_home_pose}
    
    def __repr__(self):
        return {"joint_angles":self.get_joint_angles(), "endpoint_pose":self.get_endpoint_pose_as_dict()}
    
    def __to_pose(self, pose_as_dict):
        pose = Pose()
        pose.position.x    = pose_as_dict["position"].x
        pose.position.y    = pose_as_dict["position"].y
        pose.position.z    = pose_as_dict["position"].z
        pose.orientation.x = pose_as_dict["orientation"].x
        pose.orientation.y = pose_as_dict["orientation"].y
        pose.orientation.z = pose_as_dict["orientation"].z
        pose.orientation.w = pose_as_dict["orientation"].w
        
        return pose
    
    def initialize(self):
        self.left_limb = baxter_interface.Limb("left")
        self.right_limb = baxter_interface.Limb("right")
        self.left_gripper = baxter_interface.Gripper("left")

        # rospy.loginfo("[INFO] limb dir: {}".format([i for i in dir(self.right_limb) if not i.startswith("_")]))
        # rospy.loginfo("[INFO] limb type: {}".format(type(self.right_limb)))
        # rospy.loginfo("[INFO] joint names: {}".format(self.right_limb.joint_names()))

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        rospy.loginfo("[INFO] calibrating grippers")
        self.left_gripper.calibrate(block=True)

        rospy.loginfo("[INFO] testing grippers- closing")
        self.close_gripper(block=True)

        rospy.loginfo("[INFO] testing grippers- opening")
        self.open_gripper(block=True)
    
    def terminate(self):
        pass

    def open_gripper(self, block=False):
        self.left_gripper.open(block=block)
    
    def close_gripper(self, block=False):
        self.left_gripper.close(block=block)
    
    def get_left_hand_camera_transform(self):

        self.transform_stamped = None
        result = {
            "status" : False,
            "msg" : "Did not run",
            "position":None,
            "rotation":None,
            "transform":None
        }

        try:
            self.transform_stamped = self.tf_buffer.lookup_transform("base", "left_hand_camera", rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.transform_stamped = None
            result["msg"] = "could not lookup left camera transform"
            rospy.loginfo("[ERROR] could not lookup left camera transform")

        if self.transform_stamped is None:
            return result

        result["transform"] = self.transform_stamped.transform
        result["position"] = self.transform_stamped.transform.translation.x, self.transform_stamped.transform.translation.y, self.transform_stamped.transform.translation.z
        result["rotation"] = euler_from_quaternion((self.transform_stamped.transform.rotation.x, self.transform_stamped.transform.rotation.y, self.transform_stamped.transform.rotation.z, self.transform_stamped.transform.rotation.w))
        result["status"] = True
        result["msg"] = "successful"

        return result

    def get_endpoint_pose_as_dict(self):
        return {"left":self.left_limb.endpoint_pose(), "right":self.right_limb.endpoint_pose()}
    
    def get_endpoint_pose(self):
        return {"left":self.__to_pose(self.left_limb.endpoint_pose()), "right":self.__to_pose(self.right_limb.endpoint_pose())}
    
    def get_joint_angles(self):
        return {"left":self.left_limb.joint_angles(), "right":self.right_limb.joint_angles()}
    
    def get_distance_from_current_pose(self, pose):
        x1 = pose.position.x
        y1 = pose.position.y
        z1 = pose.position.z

        end_point_pose = self.get_endpoint_pose()

        def __measure(x2, y2, z2):
            return math.sqrt( (x2-x1)**2 + (y2-y1)**2 + (z2-z1)**2 )
        
        result = {
            "left"  : __measure(end_point_pose["left"].position.x, end_point_pose["left"].position.y, end_point_pose["left"].position.z),
            "right" : __measure(end_point_pose["right"].position.x, end_point_pose["right"].position.y, end_point_pose["right"].position.z)
            }
        return result
    
    def move_to_neutral(self, which_arm="both"):
        if which_arm in ["left", "both"]:
            self.left_limb.move_to_neutral()

        if which_arm in ["right", "both"]:
            self.right_limb.move_to_neutral()

        return True
    
    def move_to_home(self, which_arm="both", wait=True):
        # self.move_to("left", self.home_pose["left"], speed, True)
        # self.move_to("right", self.home_pose["right"], speed, True)

        if which_arm in ["left", "both"]:
            trajectory_builder = JointTrajectoryBuilder("left", self.left_limb.joint_names())
            trajectory_builder.add_joint_values(self.home_joint_angles["left"], 10)
            trajectory_builder.start()

            if wait:
                trajectory_builder.wait()
        
        if which_arm in ["right", "both"]:
            trajectory_builder = JointTrajectoryBuilder("right", self.right_limb.joint_names())
            trajectory_builder.add_joint_values(self.home_joint_angles["right"], 10)
            trajectory_builder.start()

            if wait:
                trajectory_builder.wait()

        return True
    
    def move_linear(self, which_arm, start_pose, end_pose, resolution=0.3, speed=0.02, wait=True):
        poses = []
        
        start_point = pi_point.Point(start_pose.position.x, start_pose.position.y, start_pose.position.z)
        end_point   = pi_point.Point(end_pose.position.x  , end_pose.position.y  , end_pose.position.z  )
        
        line = pi_line.Line(start_point, end_point)

        for res in range(int(resolution*100.0)):
            pose = deepcopy(start_pose)

            point = line.get_point(res / 100.0)

            pose.position.x = point.x
            pose.position.y = point.y
            pose.position.z = point.z

            poses.append(pose)
        
        poses.append(deepcopy(end_pose))
        return self.follow(which_arm, poses, speed, wait)
    
    def move_to(self, which_arm, pose, speed, wait):

        joint_values = self.pose_to_joint_values(which_arm, pose)
        if joint_values is None:
            rospy.loginfo("[INFO] could not covert pose to joint values")
            return False

        joint_names = None

        if which_arm == "left":
            joint_names = self.left_limb.joint_names()
        elif which_arm == "right":
            joint_names = self.right_limb.joint_names()

        distance_required = self.get_distance_from_current_pose(pose)[which_arm]
        time_taken = 2.0 * (distance_required / speed)
        # rospy.loginfo("[INFO] time_taken = {}".format(time_taken))
        # rospy.loginfo("[INFO] distance_required = {}".format(distance_required))
        
        trajectory_builder = JointTrajectoryBuilder(which_arm, joint_names)
        trajectory_builder.add_joint_values(joint_values, time_taken)
        trajectory_builder.start()
        
        if wait:
            trajectory_builder.wait()
            return trajectory_builder.result()
        
        return True

    def follow(self, which_arm, poses, speed, wait):

        joint_data_array = []
        for pose in poses:
            joint_values = self.pose_to_joint_values(which_arm, pose)

            if joint_values is None:
                rospy.loginfo("[INFO] could not covert pose to joint values")
                continue

            joint_data_array.append((pose, joint_values))
        
        if len(joint_data_array) <= 0:
            rospy.loginfo("[INFO] could not covert any pose to its joint values")
            return False

        joint_names = None

        if which_arm == "left":
            joint_names = self.left_limb.joint_names()
        elif which_arm == "right":
            joint_names = self.right_limb.joint_names()

        
        trajectory_builder = JointTrajectoryBuilder(which_arm, joint_names)

        total_time = 0
        for pose, joint_values in joint_data_array:
            distance_required = self.get_distance_from_current_pose(pose)[which_arm]
            total_time += 2.0 * (distance_required / speed)

            trajectory_builder.add_joint_values(joint_values, total_time)

        trajectory_builder.start()
        
        if wait:
            trajectory_builder.wait()
            return trajectory_builder.result()
        
        return True
    
    def set_joint_values(self, which_arm, joint_values):
        if which_arm == "left":
            return self.left_limb.set_joint_positions(joint_values)
        elif which_arm == "right":
            return self.right_limb.set_joint_positions(joint_values)
        
        return False
    
    def pose_to_joint_values(self, which_arm, pose):
        header = Header(stamp=rospy.Time.now(), frame_id="base")
        ik_request = SolvePositionIKRequest()
        ik_request.pose_stamp.append(PoseStamped(header=header, pose=pose))
        
        ns = "ExternalTools/{which_arm}/PositionKinematicsNode/IKService".format(which_arm=which_arm)
        ik_service = rospy.ServiceProxy(ns, SolvePositionIK)

        response = None

        try:
            response = ik_service(ik_request)
        except (rospy.ServiceException, rospy.ROSException), e:
            raise RuntimeError("Failed to get response from ik server")
            return None

        r_seeds = struct.unpack("<%dB" % len(response.result_type), response.result_type)
        if r_seeds[0] != response.RESULT_INVALID:
            limb_joints = dict(zip(response.joints[0].name, response.joints[0].position))
            return limb_joints
        else:
            return None
        
        return None


def main():
    rospy.init_node("baxter_middleman")

    m = MotionController()
    m.initialize()

    end_point_pose = m.get_endpoint_pose()
    current_joint_angles = m.get_joint_angles()

    # rospy.loginfo("[INFO] baxter is at {}".format(end_point_pose))
    # rospy.loginfo("[INFO] and joint angles are {}".format(current_joint_angles))
    # rospy.loginfo("[INFO] calculated_joint_angles are {}".format(calculated_joint_angles))

    m.move_to_neutral()


if __name__ == "__main__":
    main()