#!/usr/bin/emv python

import os, sys
sys.path.insert(0,os.path.dirname(os.path.realpath(__file__)))

from server import *

def __log(status="INFO", message=""):
    print("[{status}] {message}".format(status=status, message=message))

def main():
    __log("Starting web server")
    app.run(HOST, PORT)

    time.sleep(5)
    __log("Server should be running")

if __name__ == "__main__":
    main()