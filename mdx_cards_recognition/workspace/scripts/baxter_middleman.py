#!/usr/bin/python3

import cv2
import json
import urllib
import numpy as np

class DataBridge(object):
    def __init__(self, server_url="http://localhost:8000"):
        self.urls = {
            "status" : "/status",
            "get_current_head_view"  : "/get_current_head_view",
            "get_current_hand_view"  : "/get_current_hand_view",
            "get_current_cards_view" : "/get_current_cards_view",
            "search_for_card_bin"    : "/search_for_card_bin",
            "get_card_bin_status"    : "/get_card_bin_status",
            "cancel_search_for_card_bin" : "/cancel_search_for_card_bin",
            "pickup_card_from_slot"  : "/pickup_card_from_slot"
        }

        self.server_url = server_url
    
    def __fetch_data_from_server(self, location):

        location = "{}{}".format(self.server_url, location)

        try:
            html = urllib.request.urlopen(location, timeout=5)
            return json.loads(html.read().decode("utf-8"))
        except:
            response = {
                "status"     : False,
                "error_code" : -404,
                "error_msg"  : "Error fetching data from {}".format(location)
            }
            return response
    
    def __fetch_image_from_server(self, location):
        location = "{}{}".format(self.server_url, location)

        response = {
            "status"     : False,
            "error_code" : -404,
            "error_msg"  : "Error fetching data from {}".format(location),
            "img"        : None
        }

        html = None

        try:
            html = urllib.request.urlopen(location, timeout=10)
        except:
            return response

        img = None

        try:
            img_arr = np.fromstring(html.read(), np.uint8)
            img = cv2.imdecode(img_arr, cv2.IMREAD_COLOR)
        except:
            response["error_code"] = -1
            response["error_msg"] = "Error decoding image from server"

            return response
        
        response["status"] = True
        response["error_msg"] = ""
        response["error_code"] = 0
        response["img"] = img

        return response
    
    def __post_data_to_server(self, location, data_as_dict):
        
        location = "{}{}".format(self.server_url, location)

        data = json.dumps(data_as_dict)
        data = str(data).encode("utf-8")

        response = {
            "status"     : False,
            "error_code" : -404,
            "error_msg"  : "Error posting data to {}".format(location),
            "data"        : data_as_dict
        }

        req = urllib.request.Request(location, data)

        try:
            html = urllib.request.urlopen(req, timeout=10)
        except:
            return response
        
        response["status"] = True
        response["error_msg"] = ""
        response["error_code"] = 0
        response["data"] = data_as_dict

        return response
    
    def set_server_url(self, server_url="http://localhost:8000"):
        self.server_url = server_url
    
    def fetch_server_status(self):
        return self.__fetch_data_from_server(self.urls["status"])
    
    def fetch_image_from_hand_cam(self):
        return self.__fetch_image_from_server(self.urls["get_current_hand_view"])
    
    def fetch_image_from_head_cam(self):
        return self.__fetch_image_from_server(self.urls["get_current_head_view"])
    
    def fetch_current_cards_view(self):
        return self.__fetch_image_from_server(self.urls["get_current_cards_view"])
    
    def trigger_search_for_cards_bin(self):
        return self.__fetch_data_from_server(self.urls["search_for_card_bin"])
    
    def abort_search_for_cards_bin(self):
        return self.__fetch_data_from_server(self.urls["cancel_search_for_card_bin"])
    
    def fetch_cards_bin_status(self):
        return self.__fetch_data_from_server(self.urls["get_card_bin_status"])
    
    def pickup_card(self, card_slot_number):
        return self.__fetch_data_from_server("{0}/{1}".format(self.urls["pickup_card_from_slot"], card_slot_number))
    
