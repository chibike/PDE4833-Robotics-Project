#!/usr/bin/python2

import os, sys
sys.path.insert(0,os.path.dirname(os.path.realpath(__file__)))

import rospy
from server import *

# PORT = rospy.get_param("/web_interface_node/port", 80)

def __log(status="INFO", message=""):
    rospy.loginfo("[{status}] {message}".format(status=status, message=message))

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

