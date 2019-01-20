#!/usr/bin/env python

import rospy
import signal
from std_msgs.msg import Bool, Int32
from app_man import CardSearchRoutine
from card_bin import CardBinPositionSolver
from motion_controller import MotionController
from media_access_manager import MediaAccessManager

rospy.init_node("baxter_middleman")

card_bin_solver = CardBinPositionSolver()
media_access_object = MediaAccessManager()
motion_control_object = MotionController()
card_search_manager = CardSearchRoutine(motion_control_object, media_access_object, card_bin_solver)

def on_shutdown_received():
    rospy.loginfo("terminating the motion controller")
    
    card_search_manager.terminate()
    media_access_object.terminate()
    motion_control_object.terminate()
    card_bin_solver.terminate()

rospy.on_shutdown(on_shutdown_received)

media_access_object.initialize()
motion_control_object.initialize()
card_bin_solver.initialize()

# initialize {@media_access_object, and @motion_control_object} before ...
card_search_manager.initialize()

#TODO: add flag to check if the robot is ready

rospy.loginfo("[INFO] waiting for a few seconds")
rospy.sleep(10)

rospy.loginfo("[INFO] moving to home pose")
motion_control_object.move_to_home(which_arm="left", wait=False)

rospy.loginfo("[INFO] moving to neutral pose")
motion_control_object.move_to_neutral(which_arm="right")

rospy.loginfo("[INFO] waiting for a few more seconds")
rospy.sleep(3)

rospy.loginfo("[INFO] ready for action")

def start_search(msg):
    if msg.data:
        card_search_manager.start_search()

def cancel_search(msg):
    if msg.data:
        card_search_manager.cancel_search()

def pickup_card(msg):
    slot_id = msg.data
    rospy.loginfo("[INFO] picking up your card from slot {}".format(slot_id))
    card_search_manager.pickup_card(slot_id)

pickup_card_sub = rospy.Subscriber("card_search_routine/pickup_card", Int32, pickup_card)
card_search_routine_start_sub  = rospy.Subscriber("card_search_routine/start",   Bool, start_search)
card_search_routine_cancel_sub = rospy.Subscriber("card_search_routine/cancel",  Bool, cancel_search)
card_search_manager.status_change_publisher = rospy.Publisher("card_search_routine/status", Bool, queue_size=10)
card_search_manager.publish_status()

def main():
    card_search_manager.start_image_viewer()
    rospy.spin()

try:
    main()
except KeyboardInterrupt:
    rospy.loginfo("keyboard interrupt detected")

on_shutdown_received()