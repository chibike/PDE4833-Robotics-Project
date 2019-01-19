#!/usr/bin/env python

import rospy
import signal
import baxter_interface
from sensor_msgs.msg import Image


rospy.init_node("my_cam")
display_pub= rospy.Publisher('/robot/xdisplay',Image)
def callback(msg):
        """
            Sends the camera image to baxter's display
        """             
        display_pub.publish(msg)

def close_cameras():
    rospy.loginfo("[INFO] closing cameras 2")

camera_name = "left_hand_camera"
sub = rospy.Subscriber('/cameras/' + camera_name + "/image", Image,callback,None,1)

rospy.on_shutdown(close_cameras)

try:
    rospy.spin()
except KeyboardInterrupt:
    rospy.loginfo("[INFO] keyboard interrupt detected")

rospy.loginfo("[INFO] closing cameras 1")
close_cameras()