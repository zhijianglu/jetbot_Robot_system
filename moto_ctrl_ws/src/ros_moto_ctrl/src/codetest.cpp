//
// Created by lab on 2020/7/18.
//

#include "ros/ros.h"
#include "sensor_msgs/image_encodings.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64MultiArray.h"
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <list>
#include <stdio.h>
#include "ros/ros.h"
#include "sensor_msgs/image_encodings.h"
#include "SysCtrl.h"
#include "Searcher.h"
#include <pthread.h>
#include <stdio.h>
#include <sys/time.h>
#include <string.h>

using  namespace  std;

typedef message_filters::sync_policies::ApproximateTime<CompressedImage, CompressedImage> sync_policy_classification;

//发布器
ros::Publisher gobackward_pub;
ros::Publisher headup_pub;
ros::Publisher headdown_pub;
ros::Publisher turnleft_pub;
ros::Publisher turnright_pub;
ros::Publisher turning_pub;
ros::Publisher goforward_pub;
ros::Publisher cam_turn_pub;


//控制步长
double stright_speed = 0.9;
double stright_step = 0.01;
double turn_speed = 1.2;
double turn_step = 0.007;


//测试旋转90所需要的参数
int wave_time = 0; //摆动次数，摆动以搜寻瓶子
int wave_amp = 0;  //摆动幅度
int turn_step_90 = 0; //旋转90度所需次数
int turning_direction = 0; //旋转方向  右转




void GetImgCompressed_mynteye(const sensor_msgs::CompressedImageConstPtr &msg_img_left,const sensor_msgs::CompressedImageConstPtr &msg_img_right)
{

    cv_bridge::CvImagePtr cv_ptr_img_left = cv_bridge::toCvCopy(msg_img_left, sensor_msgs::image_encodings::MONO8);
    Mat current_left_img = cv_ptr_img_left->image;

    cv_bridge::CvImagePtr cv_ptr_img_right = cv_bridge::toCvCopy(msg_img_right, sensor_msgs::image_encodings::MONO8);
    Mat current_right_img = cv_ptr_img_right->image;
    Mat hcat;

    cv::hconcat(current_left_img,current_right_img,hcat);
    cv::imshow("left_and_right", hcat);
    int key = cv::waitKey(1);

    if(turn_step_90!=0){
        std_msgs::Float64MultiArray ctl_msg;
        ctl_msg.data = std::vector<double>{turning_direction * turn_step, turn_speed};
        turning_pub.publish(ctl_msg);
        turn_step_90--;
    }
}

void command_thd(){
    while(true){
        string inputLine;
        vector<double> param;
        getline(cin, inputLine);
        stringstream ss(inputLine);
        int word;
        while(ss >> word){//>>遇到空格返回整型给word
            param.push_back(word);
        }
        std_msgs::Float64MultiArray ctl_msg;

//        cout<<param[1]<<","<< param[2]<<endl;

        switch ((int)param[0])
        {
            case 0:{  //前进
                ctl_msg.data = std::vector<double>{stright_step, stright_speed};
                goforward_pub.publish(ctl_msg);
                cout << "robot go tracking:" << "step:" << stright_step << " speed:" << stright_speed << endl;

                break;
            }
            case 1:{  //机身旋转
                ctl_msg.data = std::vector<double>{turn_step*param[1], turn_speed};
                turning_pub.publish(ctl_msg);
                cout << "robot turning:" << "step:" << turn_step * param[1] << " speed:" << turn_speed << endl;
                break;
            }
            case  2:{  //相机旋转

                ctl_msg.data = std::vector<double>{param[1], param[2]};
                cam_turn_pub.publish(ctl_msg);
                cout << "camera turning: position: " << "( x,y ) = (" << param[1] << "," << param[2] << ")" << endl;
                break;
            }

                //todo 测试机体旋转模式
            case 10:{
                //反复发送模式,命令:  模式选择  控制左右（-1 1）  迭代次数
                double scale_cam_body = -(1300.0/25.0);
                double cam_turn_val = scale_cam_body*param[1]*param[2];

                if(param[1]<0) param[2] = param[2]/ (51.0/56.0);
                ctl_msg.data = std::vector<double>{turn_step*param[1], turn_speed};

                for(int i = 0; i<param[2] ;i++){
                    turning_pub.publish(ctl_msg);
                    cout << "step by step turning:" << "step" << i << ": " << turn_step * param[2] << " speed:"
                         << turn_speed << endl;
//                    usleep(50000);
                }
                cout<<"cam should turn:"<<cam_turn_val<<endl;

                break;
            }

            case 20:{
                //反复发送模式,命令:  模式选择  控制左右（-1 1）  迭代次数
                double scale_cam_body = -(1300.0/25.0);
                double cam_turn_val = scale_cam_body*param[1]*param[2];

                if(param[1]<0) param[2] = param[2]/ (51.0/56.0);//左右换算，因为阻尼不同

                ctl_msg.data = std::vector<double>{turn_step*param[1]*param[2], turn_speed};
                turning_pub.publish(ctl_msg);
                cout << "one step turning: step" << turn_step * param[2] << " speed:" <<endl;
                cout << "cam should turn:" << cam_turn_val << endl;
                break;
            }

            case 30:{
                //反复发送模式,命令:  模式选择  控制左右（-1 1）  迭代次数
                turn_step_90 = param[1]; //旋转90度所需次数
                turning_direction = param[2]; //旋转方向  右转
                wave_time = param[3]; //旋转方向  右转
                cout << "command turning: step " << turn_step_90
                     << " searching_direction:" << turning_direction << endl;

                break;
            }

            default:
                break;
        }
    }
}

//测试转向电机的函数
int main(int argc, char **argv){

    ros::init(argc, argv, "jetbot_ctrl");
    ros::NodeHandle n;

    gobackward_pub = n.advertise<std_msgs::Float64MultiArray>("/jetbot_ctrl/gobackward", 10);
    headup_pub =  n.advertise<std_msgs::Float64MultiArray>("/jetbot_ctrl/headup", 10);
    headdown_pub = n.advertise<std_msgs::Float64MultiArray>("/jetbot_ctrl/headdown", 10);
    turnleft_pub  =   n.advertise<std_msgs::Float64MultiArray>("/jetbot_ctrl/turnleft", 10);
    turnright_pub =  n.advertise<std_msgs::Float64MultiArray>("/jetbot_ctrl/turnright", 10);

    turning_pub =  n.advertise<std_msgs::Float64MultiArray>("/jetbot_ctrl/turning_angle", 10);
    goforward_pub =  n.advertise<std_msgs::Float64MultiArray>("/jetbot_ctrl/goforward", 10);
    cam_turn_pub = n.advertise<std_msgs::Float64MultiArray>("/jetbot_ctrl/camturn", 10);

    std::string leftimage_topic_compressed = "/mynteye/left_rect/image_rect/compressed";  //订阅图像
    std::string rightimage_topic_compressed = "/mynteye/right_rect/image_rect/compressed";  //订阅图像

    message_filters::Subscriber<CompressedImage> left(n, leftimage_topic_compressed, 1);
    message_filters::Subscriber<CompressedImage> right(n, rightimage_topic_compressed, 1);
    message_filters::Synchronizer<sync_policy_classification> sync(sync_policy_classification(100), left, right);
    sync.registerCallback(boost::bind(GetImgCompressed_mynteye,_1, _2));
    std::thread t1(command_thd);

    ros::spin();

}
