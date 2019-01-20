
# MDX PDE4833 CARDS RECOGNITION WITH Baxter - Visuomotor Control with Baxter - Computer Vision Exercise
## Disclaimer
This module was written as part of my submission for Middlesex University's 2018-2019 Advanced Robotics assignment.

## Note
This script is intended for GOOD and not for EVIL
Please do not use in any negative context, at least not without prior consent from
the author of this project.

For information on how this app works check the docs folder

## Author
Name: Chibuike Okpaluba

Portfolio: https://www.chibuikepraise.com/

Email: co607@live.mdx.ac.uk

Subject: MDX Cards Advanced Robotics Projects 2018

Repo: https://github.com/chibike/mdx_cards_recognition

## Hardware Requirements
1. Baxter
2. One parallel gripper
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

# Description
This repo/project has been split into two sections:

### Baxter's workspace
The section contains the scripts to move baxter around its workspace, as well scripts to search for the card bin using aruco markers.

python scripts written to recognize four of the cards suits in a standard card deck. The programs can receive an input from a camera or folder and reliably identify the suits of the card placed in front of it (one at a time).

The recognition process is based on a preprocess-identify computing approach using several CNNs to correctly classify the presented card.

The CNN models used are:
1. ShallowNet,
2. LeNet, and
3. MiniVGGNet.

During the research for this assignment, a KNN based neural network (nn) was used but has not been included in this version.

These models were created based on the description in the pyimagesearch deeplearning Book by Dr. Adrian Rosebrock.

### Based on the Rubric
###### General Quality of Performed Task

The trained models are capable of recognizing the different card suits, and their performance was greatly improved by combining several filters using factors such as axes ratios, minimum and maximum area, and the mean probability for the classified contours.

The combination of these techniques (including the thresholding method) improves its overall resistance to noise (light, etc.). However, the proposed system would perform better in a controlled environment.

# Version
v1.0

# How to Run
### Using App
Recommended for running live demonstrations

```
python workspace/scripts/use_app.py
```

# Questions and Answers
For more information please email the author of this project
