
# MDX PDE4833 CARDS RECOGNITION with BAXTER 
###### Visuomotor Control with Baxter
###### Computer Vision Exercise

## Disclaimer
This module was written as part of my submission for Middlesex University's 2018-2019 Advanced Robotics assignment.

## Note
This script is intended for GOOD and not for EVIL
Please do not use in any negative context, at least not without prior consent from
the author of this project.

## Author
Name: Chibuike Okpaluba

Portfolio: https://www.chibuikepraise.com/

Email: co607@live.mdx.ac.uk

Subject: MDX Cards Advanced Robotics Projects 2018

Repo: https://github.com/chibike/PDE4833-Robotics-Project.git


# Version
v1.0


## Hardware Requirements
1. Baxter
2. Parallel gripper (on Baxter's left arm)
3. The cards stand used
4. Classic playing cards with only FOUR suits (Clubs, Diamonds, Hearts, and Spades).
   Special cards are not included.
5. Optional - webcam or camera

## Software Requirements
1. ROS (see http://wiki.ros.org/kinetic/Installation)
2. Baxter's software
3. Ubuntu 16.04 or 14.04
4. Python3 (required to run the app)
5. Graphviz, Imutils, Keras, Matplotlib, Numpy, OpenCV, Pillow, Pydot, PyQt5, Scipy, Sklearn, Skimage, Tensorflow

# Installation (Software)
To install Baxter's software suite on ubuntu kinetic please follow the instructions below:
```
# download the installation script
wget https://raw.githubusercontent.com/chibike/shell_scripts/master/install_baxter_sim_kinetic.sh

# change its execution mode
chmod +x install_baxter_sim_kinetic.sh

# run the file
./install_baxter_sim_kinetic.sh

# edit your baxter_ws/baxter.sh file
# use values below
baxter_hostname="011312P0003.local"
ros_version="kinetic"

# check your ip address (using ifconfig) and update the line
your_ip="192.168.XXX.XXX"
```

To install the required python packages
```
sudo apt-get update

# most python packages are no longer supported for python versions < 3.5
sudo apt-get install python3.5
sudo apt-get install python3-pip python-pip python3-tk

pip3 install --user numpy scipy matplotlib jupyter pandas sympy nose imutils pyqt5 Pillow scikit-learn scikit-image
pip3 install --user opencv-python
pip3 install --user opencv-contrib-python

pip3 install --user tensorflow
python3 -c "import tensorflow as tf; tf.enable_eager_execution();print(tf.reduce_sum(tf.random_normal([1000, 1000])))"
pip3 install --user keras

sudo apt-get install graphviz

pip3 install --user graphviz==0.5.2
pip3 install --user pydot-ng==1.0.0
pip3 install --user pydot
```

# How to Run
### Setting Up Baxter
Ensure that
1. Baxter is in a suitable environment with a least 2m^2 of space
2. The emergency button is not pressed
3. Baxter is connected to a network switch or router
4. Baxter is connected to a wall socket

If the requirements above are met, then
1. Proceed to turn Baxter on using the white power button located on its waist
2. Wait for Baxter to boot

### Setting Up your Computer
Ensure that
1. Your computer has the required software
2. Your computer is connected to the same network switch/router as Baxter
3. You have updated the `your_ip` field in baxter_ws/baxter.sh

If using a **virtual machine**, you should activate NAT Bridge connection for your LAN port

### Launching ROS
```
# change directory to ros workspace
cd PDE4833-Robotics-Project/baxter_cards_ws

# update your ip address if you haven't done so
nano baxter.sh

# if running for the first time, run
catkin_make

# connect to baxter
./baxter.sh

# start the script
roslaunch baxter_cards_launcher cards_bot.launch
```

### Launching the User Interface

Please note that the user interface is bundled seperately because it requires python3 and Baxter uses python2
This app can run on any computer within the same Local Area Network (LAN) as Baxter (or the main computer controlling Baxter)

```
# change directory to the app's location
cd PDE4833-Robotics-Project/mdx_cards_recognition/workspace/scripts

# disable ROS's default python paths
unset PYTHONPATH

# start the user interface
python3 app.py
```

# Description
**For information on how this app works check the [docs](https://github.com/chibike/PDE4833-Robotics-Project/tree/master/docs) folder**

This project showcases a program for interacting with objects in a workspace using Baxter's camera and parallel gripper. It also includes an App designed to detect/classify the objects in the workspace.

The object to be detected is a playing cards tray, designed to hold four regular sized cards, and the objects to be manipulated are the cards in the card tray.

This repo/project has been split into two sections:
1. Baxter control and object interaction,
2. User interface and object classification.

### Baxter Control and Object Interaction
### User Interface and Object Classification
###### User Interface


###### Object Classification
This repo/project contains python scripts written to recognize four of the cards suits in a standard card deck. The programs can receive an input from a camera or folder and reliably identify the suits of the card placed in front of it (one at a time).

The recognition process is based on a preprocess-identify computing approach using several CNNs to correctly classify the presented card.

The CNN models used are:
1. ShallowNet,
2. LeNet, and
3. MiniVGGNet.

These models were created based on the description in the pyimagesearch deeplearning Book by Dr. Adrian Rosebrock.

The section contains the scripts to move baxter around its workspace, as well scripts to search for the card bin using aruco markers.

python scripts written to recognize four of the cards suits in a standard card deck. The programs can receive an input from a camera or folder and reliably identify the suits of the card placed in front of it (one at a time).

### Based on the Rubric
###### System Performance
The overall performance of the program presented is pretty good as it achieves the goal of the task. Baxter can correctly estimate the position of the card tray in the XY-plane (relative to its base). It can also accurately recognize the cards (use LeNet for best results) in the card tray [mdx_cards](https://github.com/chibike/PDE4833-Robotics-Project/blob/master/mdx_cards_recognition/workspace/README.md).

The card tray can hold four cards, therefore the program presented can detect and locate up to four cards.

###### Problem Analysis and Algorithm
The algorithm used for locating and classifying the cards was carefully selected to achieve the best results.

The card bin has aruco markers placed on each of its corners, the help in the calculation of the card tray's pose of even in poor camera conditions.

It is more efficient to use markers to estimate the pose of the card tray, rather than the card's suit.

Furthermore, the approach to card recognition used achieves a high classification accuracy even in poor/harsh lighting environments.

The tasks were divided into several blocks (classes, nodes, ...) with each performing a unique task but combined together to create a viable solution.

###### Implementation
The solution presented shows a correct implementation of the components below:
1. Sensory processing (from Baxter's camera)
2. Motor control (includes a well-thought control library written for Baxter which also includes speed control [see: motion_controller.py](https://github.com/chibike/PDE4833-Robotics-Project/blob/master/baxter_cards_ws/src/motion_controller/scripts/motion_controller.py))

# Questions and Answers
For more information please email the author of this project
