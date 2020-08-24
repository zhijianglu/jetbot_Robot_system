#!/usr/bin/python3
# those for ctrl thread
from jetbot import Robot
import rospy
from moving_function import *
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Header
from std_msgs.msg import Float32MultiArray


import cv2
import numpy as np
import traitlets
from IPython.display import display
import ipywidgets.widgets as widgets
from jetbot import Camera, bgr8_to_jpeg
from jetbot import Robot
import torch.nn.functional as F
import time

# program discribe
# 1. this is for host control program ,just control motors

# those for image publisher
import rospy


def main():

    # those are moto ctrl loop
    print('listen to ctrl command topic')
    rospy.Subscriber("/jetbot_ctrl/goforward", Float64MultiArray, goforward_callback)
    rospy.Subscriber("/jetbot_ctrl/gobackward", Float64MultiArray, gobackward_callback)
    rospy.Subscriber("/jetbot_ctrl/headdown", Float64MultiArray, headdown_callback)
    rospy.Subscriber("/jetbot_ctrl/headup", Float64MultiArray, headup_callback)
    rospy.Subscriber("/jetbot_ctrl/turnleft", Float64MultiArray, turnleft_callback)
    rospy.Subscriber("/jetbot_ctrl/turnright", Float64MultiArray, turnright_callback)
    rospy.spin()


if __name__ == '__main__':
    main()

