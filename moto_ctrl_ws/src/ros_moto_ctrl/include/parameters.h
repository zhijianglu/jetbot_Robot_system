//
// Created by lab on 2020/7/18.
//

#ifndef PARAMETERS_H
#define PARAMETERS_H
#include "ros/ros.h"
#include "sensor_msgs/image_encodings.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64MultiArray.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>

extern ros::Publisher goforward_pub;
extern ros::Publisher gobackward_pub;
extern ros::Publisher turnleft_pub;
extern ros::Publisher turnright_pub;
extern ros::Publisher headup_pub;
extern ros::Publisher headdown_pub;





void setup_param(ros::NodeHandle& n){
	goforward_pub =  n.advertise<std_msgs::Float64MultiArray>("/jetbot_ctrl/goforward", 10);
	gobackward_pub = n.advertise<std_msgs::Float64MultiArray>("/jetbot_ctrl/gobackward", 10);
	turnleft_pub  =   n.advertise<std_msgs::Float64MultiArray>("/jetbot_ctrl/turnleft", 10);
	turnright_pub =  n.advertise<std_msgs::Float64MultiArray>("/jetbot_ctrl/turnright", 10);
	headup_pub =  n.advertise<std_msgs::Float64MultiArray>("/jetbot_ctrl/headup", 10);
	headdown_pub = n.advertise<std_msgs::Float64MultiArray>("/jetbot_ctrl/headdown", 10);

}



#endif //PARAMETERS_H
