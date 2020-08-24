//
// Created by lab on 2020/7/18.
//
#include "parameters.h"


ros::Publisher goforward_pub;
ros::Publisher gobackward_pub;
ros::Publisher turnleft_pub;
ros::Publisher turnright_pub;
ros::Publisher headup_pub;
ros::Publisher headdown_pub;
double test_value;

void setup_param(ros::NodeHandle& n);
