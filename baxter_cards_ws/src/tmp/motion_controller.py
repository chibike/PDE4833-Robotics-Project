#!/usr/bin/env python

import sys
import copy
import math
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler

rospy.set_param("/use_sim_time", True)

workarea_limits = {
    "x" : (0.24 ,  0.41),
    "y" : (-0.12,  0.12),
    "z" : (0.25 ,  0.5)
}

orientation_limits = {
    "r" : (-10*(math.pi/180.0), -10*(math.pi/180.0)),
    "p" : (math.pi/2.0, math.pi/2.0),
    "y" : (0.0, 0.0)
}

def constrainf(self, x, x_min, x_max):
    return min(x_max, max(x, x_min))

class MotionController(object):
    def __init__(self, group_name="", boundaries=(workarea_limits, orientation_limits)):
        self.robot  = None
        self.scene  = None
        self.group  = None

        self.group_name = group_name
        self.display_trajectory_publisher = None

        self.target = geometry_msgs.msg.Pose()
    
    def __set_position(self, position=(0,0,0), orientation=(0,0,0)):
        self.target = geometry_msgs.msg.Pose()
        orientation = quaternion_from_euler(*orientation)
        self.target.orientation.x = orientation[0]
        self.target.orientation.y = orientation[1]
        self.target.orientation.z = orientation[2]
        self.target.orientation.w = orientation[3]
        self.target.position.x = position[0] # towards you
        self.target.position.y = position[1] # sidewards
        self.target.position.z = position[2] # up-down

        # TODO: self.target = self.__validate_target()
    
    def __execute_plan(self, plan=None, max_tries=5, wait=True):
        completed = False

        for i in range(max_tries):
            completed = self.group.go(wait=wait)
            
            if completed:
                break
        
        return completed
    
    def initialize(self):
        moveit_commander.roscpp_initialize(sys.argv)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group  = moveit_commander.MoveGroupCommander(self.group_name)

        # self.group.allow_replanning(True)
        # self.group.set_planning_time(20)
        # self.group.set_max_velocity_scaling_factor(1)
        # self.group.set_max_acceleration_scaling_factor(1)

        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=10)
        rospy.sleep(3)

    def terminate(self):
        try:
            return self.group.stop()
        except AttributeError:
            if self.group is None:
                return True

        return False
    
    def publish_trajectory_display_info(self, plan):
        display_trajectory_msg = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory_msg.trajectory_start = self.robot.get_current_state()
        display_trajectory_msg.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory_msg)
    
    def overwrite_trajectory_speed(self, trajectory, speed=2.0):
        new_trajectory = moveit_msgs.msg.RobotTrajectory()
        new_trajectory.joint_trajectory = trajectory.joint_trajectory
        n_joints = len(trajectory.joint_trajectory.joint_names)
        n_points = len(trajectory.joint_trajectory.points)

        for i in range(n_points):
            new_trajectory.joint_trajectory.points[i].time_from_start = trajectory.joint_trajectory.points[i].time_from_start / speed
            new_trajectory.joint_trajectory.points[i].velocities = tuple([(j * speed) for j in trajectory.joint_trajectory.points[i].velocities])
        
        return new_trajectory
    
    def home(self):
        return self.move_to((0,0,0),speed=1,wait=True)
    
    def move_to(self, position, orientation, speed, wait=True):
        self.__set_position(position, orientation, wait)

        trajectory = None

        self.group.clear_pose_targets()
        self.group.set_pose_target(self.target)
        trajectory = self.group.plan()
        
        trajectory = self.overwrite_trajectory_speed(trajectory, speed)
        self.publish_trajectory_display_info(trajectory)

        return self.__execute_plan(trajectory, wait=wait)
    
    def follow(self, positions, orientations, speed, interpolate_resolution=0.3, jump_threshold=0.0, wait=True):
        
        waypoints = []
        for i in range( min(len(positions), len(orientations)) ):
            # ensure that every position has an orientation (using min len)

            self.__set_position(positions[i], orientations[i])
            waypoints.append( copy.deepcopy(self.target) )
        
        success = False
        trajectory = None
        ratio_of_trajectory_planned = 0.0

        self.group.clear_pose_targets()
        self.group.set_start_state_to_current_state()
        trajectory, ratio_of_trajectory_planned = self.group.compute_cartesian_path(waypoints, interpolate_resolution, jump_threshold, avoid_collisions=False)
        
        self.publish_trajectory_display_info(trajectory)
        success = self.__execute_plan(trajectory, wait=wait)

        return success, ratio_of_trajectory_planned
    
    def set_joint_angle(self, joint_index, angle_rad, wait=True):
        trajectory = None

        self.group.clear_pose_targets()
            
        joint_values = self.group.get_current_joint_values()
        joint_values[joint_index] = angle_rad

        self.group.set_joint_value_target(joint_values)
        trajectory = self.group.plan()
        
        self.publish_trajectory_display_info(trajectory)
        return __execute_plan(trajectory, wait=wait)
    
    def set_joint_angles(self, angles_rad, wait=True):
        trajectory = None

        self.group.clear_pose_targets()
            
        self.group.set_joint_value_target(angles_rad)
        trajectory = self.group.plan()
        
        self.publish_trajectory_display_info(trajectory)
        return __execute_plan(trajectory, wait=wait)
    
    def get_joint_angles(self):
        return self.group.get_current_joint_values()
    
    def get_current_pose(self):
        return self.group.get_current_pose().pose

        

def test():
    rospy.init_node("motion_controller_node", anonymous=False)

    controller = MotionController("both_arms")
    controller.initialize()

    rospy.sleep(2)

    rospy.loginfo("[INFO] joint angles {}".format(controller.get_joint_angles()))

    controller.terminate()


if __name__ == "__main__":
    test()