#!/usr/bin/python3
from jetbot import Robot
import time
import rospy
from moving_function import *

from std_msgs.msg import Float64MultiArray 

	
def listener(): 
	rospy.init_node('listener', anonymous= True)
	rospy.Subscriber("/jetbot_ctrl/goforward", Float64MultiArray,  goforward_callback)
	rospy.Subscriber("/jetbot_ctrl/gobackward", Float64MultiArray, gobackward_callback)
	rospy.Subscriber("/jetbot_ctrl/headdown", Float64MultiArray,  headdown_callback)
	rospy.Subscriber("/jetbot_ctrl/headup", Float64MultiArray, headup_callback)
	rospy.Subscriber("/jetbot_ctrl/turnleft" , Float64MultiArray,  turnleft_callback)
	rospy.Subscriber("/jetbot_ctrl/turnright", Float64MultiArray,  turnright_callback)
	rospy.spin() 



def main():
    listener()

if __name__ == '__main__':
    main()

