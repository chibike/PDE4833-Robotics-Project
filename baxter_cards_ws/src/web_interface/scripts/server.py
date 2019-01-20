#!/usr/bin/python2

import os, sys
sys.path.insert(0,os.path.dirname(os.path.realpath(__file__)))
sys.path.insert(0,"{}/../../motion_controller/scripts/".format(os.path.dirname(os.path.realpath(__file__))))

import cv2
import time
import json
import rospy
import threading
from server_config import *
from std_msgs.msg import Bool, Int32
from media_access_manager import MediaAccessManager

PORT = rospy.get_param("/web_interface_node/port", 80)

card_bin_found = False
current_cards_view = None

media_access_manager = MediaAccessManager()
media_access_manager.initialize()

card_search_routine_start_pub  = rospy.Publisher("card_search_routine/start",       Bool,  queue_size=10)
card_search_routine_cancel_pub = rospy.Publisher("card_search_routine/cancel",      Bool,  queue_size=10)
pickup_card_pub                = rospy.Publisher("card_search_routine/pickup_card", Int32, queue_size=10)

def callback(msg):
    global card_bin_found, current_cards_view

    rospy.loginfo("[INFO] heard {} from the card_search_routine status publisher".format(msg.data))
    card_bin_found = msg.data

    if card_bin_found:
        current_cards_view = media_access_manager.left_hand_cam_image

card_search_routine_status_sub = rospy.Subscriber("card_search_routine/status", Bool, callback)

def start_search():
    msg = Bool()
    msg.data = True
    card_search_routine_start_pub.publish(msg)

def cancel_search():
    msg = Bool()
    msg.data = True
    card_search_routine_cancel_pub.publish(msg)

def pickup_card_func(slot_id=0):
    msg = Int32()
    msg.data = 5 - slot_id
    pickup_card_pub.publish(msg)

def search_sim():
    global card_bin_found

    def __enable_f():
        global card_bin_found
        card_bin_found = True
    
    card_bin_found = False
    t = threading.Timer(2.0, __enable_f)
    t.start()

def cancel_search_sim():
    global card_bin_found

    # Note: This might be overwritten by the search_sim sub process
    card_bin_found = False

def on_shutdown_received():
    media_access_manager.terminate()
    sys.exit(0)

def __log(status="INFO", message=""):
    rospy.loginfo("[{status}] {message}".format(status=status, message=message))

    ## logging
    # app.logger.debug("Logging debug messages")
    # app.logger.warning("Logging warnings")
    # app.logger.error("Logging erros")

@app.route("/")
def index():
    return render_template("index.html", logged_in=False)

@app.route("/status")
def status():
    response = {
        "status"     : True,
        "error_code" : 0,
        "error_msg"  : ""
    }
    return json.dumps(response)

@app.route("/test")
def test():
    return "Test Successful"

@app.route("/get_current_head_view")
def get_current_head_view():
    img = cv2.imread("{}/img/placeholder.jpg".format(STATIC_PATH))
    img_str = cv2.imencode('.jpg', img)[1].tostring()
    return Response(img_str, mimetype='image/jpeg')

@app.route("/get_current_hand_view")
def get_current_hand_view():
    img = media_access_manager.left_hand_cam_image
    
    if not media_access_manager.is_valid(img):
        img = cv2.imread("{}/img/placeholder.jpg".format(STATIC_PATH))

    img_str = cv2.imencode('.jpg', img)[1].tostring()
    return Response(img_str, mimetype='image/jpeg')

@app.route("/get_current_cards_view")
def get_current_cards_view():
    img = current_cards_view
    if not media_access_manager.is_valid(img):
        img = cv2.imread("{}/img/demo_img.jpg".format(STATIC_PATH))
    
    img_str = cv2.imencode('.jpg', img)[1].tostring()
    return Response(img_str, mimetype='image/jpeg')

@app.route("/get_card_bin_status")
def get_card_bin_status():
    response = {
        "status"     : True,
        "error_code" : 0,
        "error_msg"  : "",
        "card_bin_found" : card_bin_found
    }
    return json.dumps(response)

@app.route("/pickup_card_from_slot/<slot_id>")
def pickup_card_from_slot(slot_id):

    response = {
        "status"     : True,
        "error_code" : 0,
        "error_msg"  : "picking up card from slot {}".format(slot_id),
        "slot_id" : slot_id
    }
    
    try:
        slot_id = str(slot_id).decode("utf-8")
        response["slot_id"] = slot_id
        pickup_card_func(int(slot_id))
    except:
        response["status"] = False
        response["error_code"] = -1
        response["error_msg"] = "Could not convert your slot id to a valid number"

    return json.dumps(response) 


@app.route("/search_for_card_bin")
def search_for_card_bin():
    
    # search_sim()
    start_search()

    response = {
        "status"     : True,
        "error_code" : 0,
        "error_msg"  : ""
    }
    return json.dumps(response)

@app.route("/cancel_search_for_card_bin")
def cancel_search_for_card_bin():
    
    # cancel_search_sim()
    cancel_search()

    response = {
        "status"     : True,
        "error_code" : 0,
        "error_msg"  : ""
    }
    return json.dumps(response)

@app.route("/shutdown")
@app.route("/kill_server")
def shutdown_server():
    __log("Shutting down the server ...")

    try:
        thread.exit()
    except:
        __log("Failed to exit server thread ...")

    on_shutdown_received()
    sys.exit(0)

def main():
    __log("initializing node {}".format(NODE_NAME))
    
    rospy.init_node("{}".format(NODE_NAME), anonymous=False)
    rospy.on_shutdown(shutdown_server)

    __log("Starting web server")
    thread.start_new_thread(app.run, (HOST, PORT))

    time.sleep(5)
    __log("Server should be running")

    rospy.spin()

if __name__ == "__main__":
    main()