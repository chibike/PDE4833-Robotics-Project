#!/usr/bin/env python

import rospy
import numpy as np
import baxter_interface
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class MediaAccessManager(object):
    def __init__(self):
        self.head_cam = None
        self.left_hand_cam = None
        
        self.im_display_pub = None

        self.head_cam_image = None
        self.left_hand_cam_image = None
        self.cards_image = None

        self.bridge = None
    
    def initialize(self):
        self.bridge = CvBridge()
        
        self.im_cards_pub = rospy.Publisher('/cards_image', Image, queue_size=2)
        self.im_display_pub = rospy.Publisher('/robot/xdisplay', Image, queue_size=2)
        
        rospy.Subscriber("/cameras/head_camera/image", Image, self.head_camera_callback, None, 1)
        rospy.Subscriber("/cameras/left_hand_camera/image", Image, self.left_hand_camera_callback, None, 1)
    
    def enable_cameras(self):
        self.head_cam = baxter_interface.CameraController("head_camera")
        self.left_hand_cam = baxter_interface.CameraController("left_hand_camera")

        # -- camera options
        # from url http://sdk.rethinkrobotics.com/wiki/Cameras
        # Mode | Resolution | Width | Height
        #   0        HIGH      1280    800
        #   1        HIGH       960    600
        #   2        HIGH       640    400
        #   3        HIGH       480    300
        #   4        HIGH       384    240
        #   5        HIGH       320    200
        #   6        LOW        640    400
        #   7        LOW        480    300
        #   8        LOW        384    240
        #   9        LOW        320    200
        #  10        LOW        240    150
        #  11        LOW        192    120

        image_option = 1

        # self.head_cam.open()
        self.left_hand_cam.resolution =(960, 600)
        # self.left_hand_cam.open()
    
    def terminate(self):
        if self.head_cam is not None:
            self.head_cam.close()
        
        if self.left_hand_cam is not None:
            self.left_hand_cam.close()
    
    def head_camera_callback(self, msg):
        self.head_cam_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    
    def left_hand_camera_callback(self, msg):
        self.left_hand_cam_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    
    def update_cards_image(self, cv_image):
        if not self.is_valid(cv_image):
            return False
        
        if self.im_cards_pub is not None:
            self.im_cards_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    
    def update_display(self, cv_image):
        if not self.is_valid(cv_image):
            return False
        
        if self.im_display_pub is not None:
            self.im_display_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

        # if not self.is_valid(cv_image):
        #     return False

        # img = cv_image.copy()

        # scale = min( (600.0/img.shape[0], 1024.0/img.shape[1]) )
        # cv2.resize(img, (0,0), fx=scale, fy=scale) # original rows: 400, cols: 640
        
        # if self.im_display_pub is not None:
        #     self.im_display_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
    
    def is_valid(self, cv_image):
        if not isinstance(cv_image, np.ndarray):
            return False
        
        im_shape = np.shape(cv_image)

        if len(im_shape) <= 0 or len(im_shape) > 3:
            return False
        
        if len(im_shape) == 3 and im_shape[2] != 3:
            return False

        if im_shape[0] <= 0 or im_shape[1] <= 0:
            return False
        
        return True


def main():
    import rospy
    # rospy.init_node("baxter_middleman")

    # m = MotionController()
    # m.initialize()

    # end_point_pose = m.get_endpoint_pose()
    # current_joint_angles = m.get_joint_angles()

    # rospy.loginfo("[INFO] baxter is at {}".format(end_point_pose))
    # rospy.loginfo("[INFO] and joint angles are {}".format(current_joint_angles))
    # rospy.loginfo("[INFO] calculated_joint_angles are {}".format(calculated_joint_angles))

    # m.move_to_neutral()


if __name__ == "__main__":
    main()