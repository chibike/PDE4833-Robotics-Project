#!/usr/bin/python3

r'''
----------------------------- App ----------------------------------
'''

import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0,currentdir)
sys.path.insert(0,"{}/vector_illustration_processing".format(currentdir))

LOGS_FOLDER = "{}/logs".format(currentdir)
_BA_SUIT_ICONS_LOCATION = "{}/res/icons".format(currentdir)

import re
import baxter_middleman

import pi_point
import pi_line
import pi_path

from imutils import paths
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *

from matplotlib import pyplot as plt
import numpy as np
import random
import math
import copy
import json
import time # used to calculate new image name
import cv2

from prediction.guessing   import Guessing_Cards
from prediction.shallownet import ShallowNet_Cards
from prediction.lenet      import LeNet_Cards
from prediction.minivggnet import MiniVGGNet_Cards

class BaxterImageViewer(QWidget):
    def __init__(self, parent=None):
        super(BaxterImageViewer, self).__init__(parent)

        self.baxter_app = None

        self.camera_image = None
        self.focus_image = None

        self.closing = False

        self.__is_viewing = False
        self.__capture_timer  = QTimer(self)
        self.__capture_timer.timeout.connect(self.handle_camera_update)
        self.__capture_timer.setInterval(100)

        self.init_UI()
        self.show()
    
    def closeEvent(self, event):
        self.closing = True

        if not self.baxter_app.closing:
            self.baxter_app.quit_app()

        self.disable_camera()
        event.accept()
    
    def keyPressEvent(self, event):
        key = event.key()

        if key in [Qt.Key_Q, Qt.Key_Escape]:
            self.quit_app()

    def quit_app(self):
        super(BaxterImageViewer, self).close()
    
    def init_UI(self):
        self.setWindowTitle("Image Viewer")
        self.setWindowModality(Qt.ApplicationModal)
        self.setFixedSize(525, 255)
        self.move(100, 400)

        self.camera_view_scene = QGraphicsScene()
        self.camera_view_view  = QGraphicsView(self.camera_view_scene, self)
        self.camera_view_view.resize(375, 225)
        self.camera_view_view.move(150, 0)

        self.focus_view_scene = QGraphicsScene()
        self.focus_view_view  = QGraphicsView(self.focus_view_scene, self)
        self.focus_view_view.resize(150, 225)

        section_heading = QLabel("Selected Card", self)
        section_heading.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        section_heading.setStyleSheet("QLabel {color: red; font: bold 14px;}")
        section_heading.move(30, 235)

        section_heading = QLabel("Current Cards View", self)
        section_heading.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        section_heading.setStyleSheet("QLabel {color: red; font: bold 14px;}")
        section_heading.move(270, 235)
    
    def set_baxter_app(self, baxter_app):
        self.baxter_app = baxter_app
    
    def set_focus_image(self, img):
        if not self.is_frame_valid(img):
            return False
        
        self.focus_image = img
    
    def is_frame_valid(self, frame, nchannel=3):
        if not isinstance(frame, np.ndarray):
            return False
        
        try:
            if len(frame.shape) != nchannel:
                return False
        except:
            return False
        
        return True
    
    def handle_camera_update(self):
        if not self.__is_viewing:
            return
        
        response = self.baxter_app.baxter_data_bridge.fetch_current_cards_view()
        if not response["status"]:
            print ("[ERROR] {}".format(response["error_msg"]))
            return

        frame = response["img"]

        if not self.is_frame_valid(frame):
            return
        
        self.camera_image = frame.copy()
        
        # for cropped frame
        # nh, nw = 320, 207; offset = 216
        # self.focus_image = frame[0:nh, offset:offset+nw, :]

        return self.render()
    
    def render(self):
        if not self.is_frame_valid(self.camera_image):
            return
        
        img = cv2.cvtColor(self.camera_image, cv2.COLOR_BGR2RGB)
        im_height, im_width, byte_val = img.shape

        ci_qimage = QImage(img.data, im_width, im_height, byte_val * im_width, QImage.Format_RGB888)
        ci_pix_map = QPixmap.fromImage(ci_qimage)

        self.camera_view_scene.clear()
        self.camera_view_scene.addPixmap(ci_pix_map)
        self.camera_view_view.fitInView(QRectF(0,0,im_width, im_height), Qt.KeepAspectRatio)
        self.camera_view_scene.update()

        
        if not self.is_frame_valid(self.focus_image):
            return
        
        img = cv2.cvtColor(self.focus_image, cv2.COLOR_BGR2RGB)
        im_height, im_width, byte_val = img.shape

        ci_qimage = QImage(img.data, im_width, im_height, byte_val * im_width, QImage.Format_RGB888)
        ci_pix_map = QPixmap.fromImage(ci_qimage)

        self.focus_view_scene.clear()
        self.focus_view_scene.addPixmap(ci_pix_map)
        self.focus_view_view.fitInView(QRectF(0,0,im_width, im_height), Qt.KeepAspectRatio)
        self.focus_view_scene.update()
    
    def enable_camera(self):
        if self.__is_viewing:
            return
        
        self.__is_viewing = True
        self.__capture_timer.start()
        # self.baxter_app.show_message(msg="Could not access your camera")    
    
    def disable_camera(self):
        self.__capture_timer.stop()
        self.__is_viewing = False
    
class BaxterApp(QWidget):
    def __init__(self, parent=None):
        super(BaxterApp, self).__init__(parent)

        self.closing = False
        self.baxter_image_viewer = None
        self.baxter_data_bridge = baxter_middleman.DataBridge()

        __name_pre_formater = lambda n : "{}/{}_icon.png".format(_BA_SUIT_ICONS_LOCATION, n)
        self.icons = {
            "clubs"    : __name_pre_formater(  "clubs"),
            "spades"   : __name_pre_formater( "spades"),
            "hearts"   : __name_pre_formater( "hearts"),
            "diamonds" : __name_pre_formater("diamonds")
        }

        self.classifiers = {
            "ShallowNet" : ShallowNet_Cards,
            "LeNet"      : LeNet_Cards,
            "MiniVGGNet" : MiniVGGNet_Cards
        }

        self.card_images = {
            "original" : None,
            "card_01"  : None,
            "card_02"  : None,
            "card_03"  : None,
            "card_04"  : None
        }

        self.card_suits = {
            "card_01"  : None,
            "card_02"  : None,
            "card_03"  : None,
            "card_04"  : None
        }

        self.is_connected = False
        self.is_searching_for_cards_bin = False

        self.camera_id = 0
        self.baxter_ip_address = ""
        self.current_classifier = None
        self.selected_card_view = 1

        self.__card_bin_status_tracker_timer  = QTimer(self)
        self.__card_bin_status_tracker_timer.timeout.connect(self.check_card_bin_status)
        self.__card_bin_status_tracker_timer.setInterval(1000)
        
        self.init_UI()
        self.select_classifier_cb.setCurrentIndex(0)    # set the default prediction model to a LeNet based cnn
        
        self.show()
    
    def closeEvent(self, event):
        reply = QMessageBox.question(self, "Quiting", "Do you really want to exit?", QMessageBox.Yes, QMessageBox.No)

        if reply == QMessageBox.Yes:
            self.closing = True

            if not self.baxter_image_viewer.closing:
                self.baxter_image_viewer.quit_app()

            event.accept()
        else:
            event.ignore()
    
    def keyPressEvent(self, event):
        key = event.key()

        if key in [Qt.Key_Q, Qt.Key_Escape]:
            self.quit_app()
        elif key == Qt.Key_S:
            def __save_img(card_name="card_01"):
                result = self.predict_card_from_image(self.card_images[card_name])
                if result["status"]:
                    cv2.imwrite("{}/{}_in.png".format(LOGS_FOLDER, card_name), self.card_images[card_name])
                    cv2.imwrite("{}/{}_out.png".format(LOGS_FOLDER, card_name), result["img_out"])
            
            __save_img("card_01")
            __save_img("card_02")
            __save_img("card_03")
            __save_img("card_04")

            self.show_message(title="Information", msg="Saved succesful images to {}".format(LOGS_FOLDER))
        
    def init_UI(self):
        self.setWindowTitle("MDX Baxter Cards Selector")
        self.setWindowModality(Qt.ApplicationModal)
        self.setFixedSize(650, 445)

        x_padding = 10
        y_padding = 10
        card_view_width = 150
        card_view_height = 1.5 * card_view_width
        btn_height = 3 * y_padding

        section_heading = QLabel("Detected Cards Suit", self)
        section_heading.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        section_heading.setStyleSheet("QLabel {color: red; font: bold 14px;}")
        section_heading.move(x_padding, y_padding)

        self.card_01_scene  = QGraphicsScene()
        self.card_01_view = QGraphicsView(self.card_01_scene, self)
        self.card_01_view.resize(card_view_width, card_view_height)
        self.card_01_view.move(x_padding, (y_padding*3))

        self.card_02_scene  = QGraphicsScene()
        self.card_02_view = QGraphicsView(self.card_02_scene, self)
        self.card_02_view.resize(card_view_width, card_view_height)
        self.card_02_view.move((2 * x_padding) + card_view_width, (y_padding*3))

        self.card_03_scene  = QGraphicsScene()
        self.card_03_view = QGraphicsView(self.card_03_scene, self)
        self.card_03_view.resize(card_view_width, card_view_height)
        self.card_03_view.move((3 * x_padding)  + (2 * card_view_width), (y_padding*3))

        self.card_04_scene  = QGraphicsScene()
        self.card_04_view = QGraphicsView(self.card_04_scene, self)
        self.card_04_view.resize(card_view_width, card_view_height)
        self.card_04_view.move((4 * x_padding)  + (3 * card_view_width), (y_padding*3))
        
        self.card_01_identify_btn = QPushButton("Identify", self)
        self.card_01_identify_btn.setCheckable(False)
        self.card_01_identify_btn.resize(card_view_width, btn_height)
        self.card_01_identify_btn.move(x_padding, (y_padding*4) + card_view_height)
        self.card_01_identify_btn.clicked.connect(self.render_card_01)

        self.card_02_identify_btn = QPushButton("Identify", self)
        self.card_02_identify_btn.setCheckable(False)
        self.card_02_identify_btn.resize(card_view_width, btn_height)
        self.card_02_identify_btn.move((2 * x_padding) + card_view_width, (y_padding*4) + card_view_height)
        self.card_02_identify_btn.clicked.connect(self.render_card_02)

        self.card_03_identify_btn = QPushButton("Identify", self)
        self.card_03_identify_btn.setCheckable(False)
        self.card_03_identify_btn.resize(card_view_width, btn_height)
        self.card_03_identify_btn.move((3 * x_padding)  + (2 * card_view_width), (y_padding*4) + card_view_height)
        self.card_03_identify_btn.clicked.connect(self.render_card_03)

        self.card_04_identify_btn = QPushButton("Identify", self)
        self.card_04_identify_btn.setCheckable(False)
        self.card_04_identify_btn.resize(card_view_width, btn_height)
        self.card_04_identify_btn.move((4 * x_padding)  + (3 * card_view_width), (y_padding*4) + card_view_height)
        self.card_04_identify_btn.clicked.connect(self.render_card_04)

        
        
        self.card_01_select_btn = QPushButton("Select", self)
        self.card_01_select_btn.setCheckable(True)
        self.card_01_select_btn.resize(card_view_width, btn_height)
        self.card_01_select_btn.move(x_padding, (y_padding*5) + card_view_height + btn_height)
        self.card_01_select_btn.clicked.connect(self.card_01_select_btn_callback)

        self.card_02_select_btn = QPushButton("Select", self)
        self.card_02_select_btn.setCheckable(True)
        self.card_02_select_btn.resize(card_view_width, btn_height)
        self.card_02_select_btn.move((2 * x_padding) + card_view_width, (y_padding*5) + card_view_height + btn_height)
        self.card_02_select_btn.clicked.connect(self.card_02_select_btn_callback)

        self.card_03_select_btn = QPushButton("Select", self)
        self.card_03_select_btn.setCheckable(True)
        self.card_03_select_btn.resize(card_view_width, btn_height)
        self.card_03_select_btn.move((3 * x_padding)  + (2 * card_view_width), (y_padding*5) + card_view_height + btn_height)
        self.card_03_select_btn.clicked.connect(self.card_03_select_btn_callback)

        self.card_04_select_btn = QPushButton("Select", self)
        self.card_04_select_btn.setCheckable(True)
        self.card_04_select_btn.resize(card_view_width, btn_height)
        self.card_04_select_btn.move((4 * x_padding)  + (3 * card_view_width), (y_padding*5) + card_view_height + btn_height)
        self.card_04_select_btn.clicked.connect(self.card_04_select_btn_callback)

        
        section_heading = QLabel("Controls - use LeNet for best results", self)
        section_heading.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        section_heading.setStyleSheet("QLabel {color: red; font: bold 14px;}")
        section_heading.move(x_padding, (y_padding*7) + card_view_height + (btn_height * 2))




        self.connect_app_btn = QPushButton("Connect", self)
        self.connect_app_btn.setCheckable(False)
        self.connect_app_btn.resize(card_view_width, (btn_height * 2))
        self.connect_app_btn.move(x_padding, (y_padding*9) + card_view_height + (btn_height * 2))
        self.connect_app_btn.clicked.connect(self.connect_app_btn_callback)

        self.select_classifier_cb = QComboBox(self)
        self.select_classifier_cb.addItems(list(sorted(self.classifiers.keys())))
        self.select_classifier_cb.resize(card_view_width, (btn_height * 2))
        self.select_classifier_cb.move((2 * x_padding) + card_view_width, (y_padding*9) + card_view_height + (btn_height * 2))
        self.select_classifier_cb.currentIndexChanged.connect(self.select_classifier_cb_callback)

        self.detect_cards_btn = QPushButton("Detect Cards", self)
        self.detect_cards_btn.setCheckable(False)
        self.detect_cards_btn.resize(card_view_width, (btn_height * 2))
        self.detect_cards_btn.move((3 * x_padding)  + (2 * card_view_width), (y_padding*9) + card_view_height + (btn_height * 2))
        self.detect_cards_btn.clicked.connect(self.detect_cards_btn_callback)

        self.fetch_card_btn = QPushButton("Fetch Card", self)
        self.fetch_card_btn.setCheckable(False)
        self.fetch_card_btn.resize(card_view_width, (btn_height * 2))
        self.fetch_card_btn.move((4 * x_padding)  + (3 * card_view_width), (y_padding*9) + card_view_height + (btn_height * 2))
        self.fetch_card_btn.clicked.connect(self.fetch_card_btn_callback)

        # self.w = QWidget(self)
        
        # layout = QFormLayout()
        # layout.setVerticalSpacing(10)
        # layout.setHorizontalSpacing(10)

        # section_heading = QLabel("Card Suits")
        # section_heading.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        # section_heading.setStyleSheet("QLabel {color: red; font: bold 14px;}")
        # layout.addRow(section_heading)

        # self.w.setLayout(layout)
    
    def quit_app(self):
        super(BaxterApp, self).close()
    
    def set_baxter_image_viewer(self, baxter_image_viewer):
        self.baxter_image_viewer = baxter_image_viewer
    
    def show_message(self, title="Warning!", msg="Could not read image?"):
        QMessageBox.question(self, title, msg, QMessageBox.Ok, QMessageBox.Ok)
    
    def render_card_01(self):
        result = self.predict_card_from_image(self.card_images["card_01"])
        if not result["status"]:
            self.card_suits["card_01"] = None
            self.card_01_scene.clear()
            self.card_01_scene.update()
            return

        self.card_suits["card_01"] = result["suit"]
        img = cv2.imread(self.icons[self.card_suits["card_01"]])

        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        im_height, im_width, byte_val = img.shape

        ci_qimage = QImage(img.data, im_width, im_height, byte_val * im_width, QImage.Format_RGB888)
        ci_pix_map = QPixmap.fromImage(ci_qimage)

        self.card_01_scene.clear()
        self.card_01_scene.addPixmap(ci_pix_map)
        self.card_01_view.fitInView(QRectF(0,0,im_width, im_height), Qt.KeepAspectRatio)
        self.card_01_scene.update()

        return True
    
    def render_card_02(self):
        result = self.predict_card_from_image(self.card_images["card_02"])
        if not result["status"]:
            self.card_suits["card_02"] = None
            self.card_02_scene.clear()
            self.card_02_scene.update()
            return

        self.card_suits["card_02"] = result["suit"]
        img = cv2.imread(self.icons[self.card_suits["card_02"]])

        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        im_height, im_width, byte_val = img.shape

        ci_qimage = QImage(img.data, im_width, im_height, byte_val * im_width, QImage.Format_RGB888)
        ci_pix_map = QPixmap.fromImage(ci_qimage)

        self.card_02_scene.clear()
        self.card_02_scene.addPixmap(ci_pix_map)
        self.card_02_view.fitInView(QRectF(0,0,im_width, im_height), Qt.KeepAspectRatio)
        self.card_02_scene.update()

        return True
    
    def render_card_03(self):
        result = self.predict_card_from_image(self.card_images["card_03"])
        if not result["status"]:
            self.card_suits["card_03"] = None
            self.card_03_scene.clear()
            self.card_03_scene.update()
            return

        self.card_suits["card_03"] = result["suit"]
        img = cv2.imread(self.icons[self.card_suits["card_03"]])

        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        im_height, im_width, byte_val = img.shape

        ci_qimage = QImage(img.data, im_width, im_height, byte_val * im_width, QImage.Format_RGB888)
        ci_pix_map = QPixmap.fromImage(ci_qimage)

        self.card_03_scene.clear()
        self.card_03_scene.addPixmap(ci_pix_map)
        self.card_03_view.fitInView(QRectF(0,0,im_width, im_height), Qt.KeepAspectRatio)
        self.card_03_scene.update()

        return True
    
    def render_card_04(self):
        result = self.predict_card_from_image(self.card_images["card_04"])
        if not result["status"]:
            self.card_suits["card_04"] = None
            self.card_04_scene.clear()
            self.card_04_scene.update()
            return
        
        self.card_suits["card_04"] = result["suit"]
        img = cv2.imread(self.icons[self.card_suits["card_04"]])

        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        im_height, im_width, byte_val = img.shape

        ci_qimage = QImage(img.data, im_width, im_height, byte_val * im_width, QImage.Format_RGB888)
        ci_pix_map = QPixmap.fromImage(ci_qimage)

        self.card_04_scene.clear()
        self.card_04_scene.addPixmap(ci_pix_map)
        self.card_04_view.fitInView(QRectF(0,0,im_width, im_height), Qt.KeepAspectRatio)
        self.card_04_scene.update()

        return True
    
    def is_image_valid(self, img):
        if not isinstance(img, np.ndarray):
            return False
        
        im_shape = img.shape

        if len(im_shape) != 3 or im_shape[2] != 3:
            return False
        
        if im_shape[0] <= 0 or im_shape[1] <= 0:
            return False
        
        return True
    
    def select_classifier_cb_callback(self, index):
        text = self.select_classifier_cb.currentText()

        if text == "ShallowNet":
            self.current_classifier = ShallowNet_Cards.SuitDetector()
        elif text == "LeNet":
            self.current_classifier = LeNet_Cards.SuitDetector()
        elif text == "MiniVGGNet":
            self.current_classifier = MiniVGGNet_Cards.SuitDetector()
    
    def connect_app_btn_callback(self):
        if not self.is_connected:

            def __get_address_from_user():
                baxter_ip_address, ok_pressed = QInputDialog.getText(self, "Connect with Baxter","IP Address:")
                if not ok_pressed:
                    return False, None
                
                if len(baxter_ip_address) <= 0:
                    self.show_message(msg="No address received")
                    return False, None
                
                search_pattern = r'^[0-9]{1,3}[.][0-9]{1,3}[.][0-9]{1,3}[.][0-9]{1,3}$'
                match_object = re.match(search_pattern, baxter_ip_address.strip(), re.I)

                if match_object is None:
                    self.show_message(msg="Invalid address received")
                    return False, None
                
                return True, "http://{}:8000".format(str(match_object.group()))

            def __connect(ip_address="http://127.0.0.1:8000"):
                self.baxter_ip_address = ip_address
                self.baxter_data_bridge.set_server_url(ip_address)
                return self.baxter_data_bridge.fetch_server_status()
            
            result = __connect()
            if not result["status"]:
                status, baxter_ip_address = __get_address_from_user()

                if not status:
                    return
                
                result = __connect(baxter_ip_address)

                if not result["status"]:
                    self.show_message(msg="Could not connect to Baxter at {}.\nReceived [\"{}\"] instead".format(self.baxter_ip_address, result["error_msg"]))
                    return

            self.baxter_image_viewer.enable_camera()
            self.show_message(title="Information", msg="Connected to Baxter at {}".format(self.baxter_ip_address))

            self.is_connected = True
            self.connect_app_btn.setText("Disconnect")
            self.connect_app_btn.setStyleSheet("background-color: green")

        else:
            self.baxter_image_viewer.disable_camera()

            self.is_connected = False
            self.connect_app_btn.setText("Connect")
            self.connect_app_btn.setStyleSheet("background-color: white")
    
    def update_focus_view(self):
        if self.selected_card_view == 1:
            self.baxter_image_viewer.set_focus_image(self.card_images["card_01"])
        elif self.selected_card_view == 2:
            self.baxter_image_viewer.set_focus_image(self.card_images["card_02"])
        elif self.selected_card_view == 3:
            self.baxter_image_viewer.set_focus_image(self.card_images["card_03"])
        elif self.selected_card_view == 4:
            self.baxter_image_viewer.set_focus_image(self.card_images["card_04"])
        
    def card_01_select_btn_callback(self):
        if not self.card_01_select_btn.isChecked():
            return

        self.card_02_select_btn.setChecked(False)
        self.card_03_select_btn.setChecked(False)
        self.card_04_select_btn.setChecked(False)

        self.selected_card_view = 1
        self.update_focus_view()
    
    def card_02_select_btn_callback(self):
        if not self.card_02_select_btn.isChecked():
            return

        self.card_01_select_btn.setChecked(False)
        self.card_03_select_btn.setChecked(False)
        self.card_04_select_btn.setChecked(False)

        self.selected_card_view = 2
        self.update_focus_view()
    
    def card_03_select_btn_callback(self):
        if not self.card_03_select_btn.isChecked():
            return

        self.card_01_select_btn.setChecked(False)
        self.card_02_select_btn.setChecked(False)
        self.card_04_select_btn.setChecked(False)

        self.selected_card_view = 3
        self.update_focus_view()
    
    def card_04_select_btn_callback(self):
        if not self.card_04_select_btn.isChecked():
            return

        self.card_01_select_btn.setChecked(False)
        self.card_02_select_btn.setChecked(False)
        self.card_03_select_btn.setChecked(False)

        self.selected_card_view = 4
        self.update_focus_view()
    
    def detect_cards_btn_callback(self):
        if self.is_searching_for_cards_bin:
            self.is_searching_for_cards_bin = False

            self.baxter_data_bridge.abort_search_for_cards_bin()
            self.__card_bin_status_tracker_timer.stop()

            self.detect_cards_btn.setText("Detect Cards")
            self.detect_cards_btn.setStyleSheet("background-color: white")
            self.show_message(title="Warning!", msg="Aborted search for cards bin")
        else:
            if not self.is_connected:
                self.show_message(msg="You are not connected to a server, use the connect button")
                return
            
            reply = QMessageBox.question(self, "Warning!", "This will move the Robot's arms.\nDo you wish to proceed?", QMessageBox.Yes, QMessageBox.No)
            if reply != QMessageBox.Yes:
                return

            self.is_searching_for_cards_bin = True

            self.baxter_data_bridge.trigger_search_for_cards_bin()
            self.__card_bin_status_tracker_timer.start()
            
            self.detect_cards_btn.setText("Cancel")
            self.detect_cards_btn.setStyleSheet("background-color: red")
            self.show_message(title="Information", msg="Searching for cards bin, please wait")
        
    def fetch_card_btn_callback(self):
        if not self.is_connected:
            self.show_message(msg="You are not connected to a server, use the connect button")
            return
            
        response = self.baxter_data_bridge.fetch_cards_bin_status()
        if (not response["status"]) or (not response["card_bin_found"]):
            self.show_message(title="Information", msg="Use the detect button to locate the card bin")
            return
        
        if not True in [self.card_01_select_btn.isChecked(), self.card_02_select_btn.isChecked(), self.card_03_select_btn.isChecked(), self.card_04_select_btn.isChecked()]:
            self.show_message(title="Information", msg="Please select a card")
            return
        
        reply = QMessageBox.question(self, "Warning!", "This will move the Robot's arms.\nDo you wish to proceed?", QMessageBox.Yes, QMessageBox.No)
        if reply != QMessageBox.Yes:
            return
        
        self.baxter_data_bridge.pickup_card(self.selected_card_view)
        self.show_message(title="Information", msg="Picking your card, please wait")
    
    def check_card_bin_status(self):
        response = self.baxter_data_bridge.fetch_cards_bin_status()

        if response["status"] and response["card_bin_found"]:
            self.on_card_bin_found()
    
    def on_card_bin_found(self):
        self.is_searching_for_cards_bin = False

        self.__card_bin_status_tracker_timer.stop()

        self.detect_cards_btn.setText("Detect Cards")
        self.detect_cards_btn.setStyleSheet("background-color: white")
        self.show_message(title="Information", msg="Found cards bin, will now analyze the images")

        response = self.baxter_data_bridge.fetch_current_cards_view()
        if not response["status"]:
            self.show_message(msg="Error [\"{}\"] while retrieving image".format(response["error_msg"]))
            return

        self.process_img(response["img"])

    def process_img(self, img):
        if not isinstance(img, np.ndarray):
            self.show_message(msg="Error, invalid image received for processing")
            return
        
        im_shape = img.shape

        if len(im_shape) != 3 or im_shape[2] != 3:
            self.show_message(msg="Error, received image with an unknown format")
            return
        
        if im_shape[0] <= 0 or im_shape[1] <= 0:
            self.show_message(msg="Error, image has invalid dimensions")
            return
        
        img = img[50:300,50:550,:]
        im_shape = img.shape
        
        card_width = int(im_shape[1] / 4)
        self.card_images["original"] = img.copy()
        self.card_images["card_04"] = img[:,0:card_width,:]
        self.card_images["card_03"] = img[:,1*card_width:2*card_width,:]
        self.card_images["card_02"] = img[:,2*card_width:3*card_width,:]
        self.card_images["card_01"] = img[:,3*card_width:,:]

        self.render_card_01()
        self.render_card_02()
        self.render_card_03()
        self.render_card_04()

        self.update_focus_view()

    def predict_card_from_image(self, card_img):
        result = {
            "suit"        : None,
            "number"      : None,
            "probability" : 1.0,
            "status"      : False,
            "error_code"  : -404,
            "error_msg"   : "No card found"
        }

        if not self.is_image_valid(card_img):
            return result

        frame = card_img.copy()

        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray_frame = cv2.GaussianBlur(gray_frame, (5, 5), 0)

        gray_mean  = int(np.mean(gray_frame.ravel()))
        _, gray_th  = cv2.threshold(gray_frame, gray_mean, 255, cv2.THRESH_BINARY)

        kernel = np.ones((3,3),np.uint8)
        gray_th = cv2.erode(gray_th, kernel, iterations=1)

        _, contours, _ = cv2.findContours(gray_th, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        def __get_image_in_contour(img, path, contour):
            '''
                Returns the sub-section (with padding) of the given image contained within the
                given contour, alongside its mask.
            '''

            im_h, im_w = img.shape[:2]
            mask = np.zeros((im_h, im_w))
            cv2.drawContours(mask, [contour], -1, 255, -1)

            padding = 10
            top_x = max(path.rect_info.top_left.x - padding, 0)
            bottom_x = min(path.rect_info.bottom_right.x + padding, im_w)

            top_y = max(path.rect_info.top_left.y - padding, 0)
            bottom_y = min(path.rect_info.bottom_right.y + padding, im_h)
            
            n_mask = mask[top_y:bottom_y, top_x:bottom_x]
            n_img  = img[top_y:bottom_y, top_x:bottom_x]

            return n_img, n_mask
        
        def __constrain(x, mn_x, mx_x):
            return min(mx_x, max(x, mn_x))
        
        def __determine_prediction(labels, probabilities):
            #  Given a random assortment of labels and probabilties, this function returns a list containing each label and its mean probability
            #  Don't try to comprehend how this works --> (it's magic)
            #
            # e.g takes [a, b, c, a, c, b, a, b ...], [0.1, 0.5, 0.2, 0.3, 0.4, 0.1, 0.9, 0.1 ...]
            # and returns [(a, prob), (b, prob), (c, prob)]    # something like this --> :)

            f = lambda a, b : [list(filter(lambda x: x[0] == i, sorted(list(zip(a, b)), key=lambda x: x[0]))) for i in list(set(a))]

            def __get_prediction(foo, n):
                label, preds = list(zip(*foo))
                label = label[0]
                preds = list(sorted(preds)[-n:])
                return label, sum(preds) / float(len(preds))
            
            # pre-format data
            data = f(labels, probabilities)
            n_data = []

            for datum in data:
                r = __get_prediction(datum, 3)
                n_data.append((r[0], r[1]))
            
            # rearrange the list to ensure that label with the highest probability is first
            return list(sorted(n_data, key=lambda x : x[1], reverse=True))
        
        in_img_w, in_img_h,_ = frame.shape
        
        contours_list = [contour.reshape((contour.shape[0], 2)).tolist() for contour in contours]
        paths_list    = [pi_path.Path(raw_point_data=[pi_point.Point(x=point[0], y=point[1]) for point in contour_points], is_closed=True) for contour_points in contours_list]

        filtered_paths = []
        filtered_cnts = []
        labels = []
        probs = [] 

        # do not consider contours that have areas less than {@ min_allowed_area}
        min_allowed_area = (in_img_w * in_img_h) * (500.0 / 66240.0)
        paths_attributes  = [(path, path.rect_info.area, path.rect_info.perimeter) for path in paths_list if path.rect_info.area > min_allowed_area]
        paths_list, area_list, perimeter_list = zip(*paths_attributes)

        # getting a prediction for every shape found in the image
        for path in paths_list:
            rect_info = path.rect_info

            # do not add contours with invalid an axes ratio, or area
            if path.ratio < 0.6: continue
            if rect_info.area > (in_img_w * in_img_h) * (7000.0 / 66240.0): continue

            # convert path to contour
            cnt = path.get_as_contour()

            # extract a padded section of image with the contour
            n_img, n_mask = __get_image_in_contour(frame, path, cnt)

            # THIS IS WHERE THE PREDICTION IS DONE
            if self.current_classifier is not None:
                label, prob = self.current_classifier.predict(n_img)

                if prob < 0.9: continue # Do not accept contours with probabilities < than 0.9

                labels.append(label)
                probs.append(prob)
            
            filtered_cnts.append(cnt)
            filtered_paths.append(path)
        
        preds = __determine_prediction(labels, probs)
        number_labels = ["Ace", "Two", "Three", "Four", "Five", "Six", "Seven", "Eight", "Nine", "Ten"]

        if len(preds) > 0:
            label, prob = preds[0] # select the prediction with the highest probability
            
            result["suit"]        = label
            result["number"]      = number_labels[__constrain(len(filtered_paths)-1, 0, len(number_labels)-1)]
            result["probability"] = prob
            result["status"]      = True
            result["error_code"]  = 1
            result["error_msg"]   = ""

            out_img = frame.copy()
            cv2.drawContours(out_img, filtered_cnts, -1, (0,0,255), 1)
            result["img_out"] = out_img

    
        return result


if __name__ == '__main__':
    app = QApplication(sys.argv)
    
    biv = BaxterImageViewer()
    ba  = BaxterApp()

    biv.set_baxter_app(ba)
    ba.set_baxter_image_viewer(biv)

    sys.exit(app.exec_())
