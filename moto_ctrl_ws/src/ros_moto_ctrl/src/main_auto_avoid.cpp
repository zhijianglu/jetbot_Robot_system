//
// Created by lab on 2020/7/17.
//

#include "ros/ros.h"
#include "sensor_msgs/image_encodings.h"
#include "SysCtrl.h"
#include "Searcher.h"

SysCtrl* p_sysctrl;
Searcher* p_searcher;

using namespace std_msgs;
typedef message_filters::sync_policies::ApproximateTime<CompressedImage, CompressedImage> sync_policy_classification;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "jetbot_ctrl");
	ros::NodeHandle nh;
//	p_sysctrl.reset(new SysCtrl(nh));

	ros::Subscriber prob_obstacle_sub; //来自深度图的障碍判断
	ros::Subscriber prob_obstacle_dev_sub;  //来自于jetbot测距的障碍物判断
	ros::Subscriber compreassed_depth_sub;
	ros::Subscriber raw_depth_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber camera_pose_sub;  //来自于jetbot测距的障碍物判断
    ros::Subscriber temp_sub;  //来自于jetbot测距的障碍物判断


//	std::string image_topic_compressed = "/camera/rgb/image_raw/compressed";  //订阅机载相机的节点，来自于jetbotCam
	std::string prob_obstacle_topic = "/prob_blocked";  //订阅
	std::string prob_obstacle_dev_topic = "/prob_blocked_dev";  //订阅
	std::string pose_topic = "/vins_estimator/camera_pose";  //订阅
	std::string temp_topic = "/temperature";  //订阅
    p_sysctrl = new SysCtrl(nh);
//    p_searcher = new Searcher(nh);

    p_sysctrl->readParameters("/home/lab/Project/FullRobotSystem/moto_ctrl_ws/src/ros_moto_ctrl/config/config0.yaml");

//同步接收两个图像
    std::string leftimage_topic_compressed = "/mynteye/left_rect/image_rect/compressed";  //订阅图像
    std::string rightimage_topic_compressed = "/mynteye/right_rect/image_rect/compressed";  //订阅图像

    message_filters::Subscriber<CompressedImage> left(nh, leftimage_topic_compressed, 1);
    message_filters::Subscriber<CompressedImage> right(nh, rightimage_topic_compressed, 1);
    message_filters::Synchronizer<sync_policy_classification> sync(sync_policy_classification(100), left, right);
    sync.registerCallback(boost::bind(&SysCtrl::GetImgCompressed_mynteye,p_sysctrl,_1, _2));

//    std::string depth_topic_compressed = "/mynteye/disparity/image_norm/compressed";  //订阅视差图的压缩版,测试时候使用，发现稠密度不够
//    std::string depth_topic_raw = "/mynteye/depth/image_raw";  //订阅原始深度图，但是速率不够
//    compreassed_depth_sub = nh.subscribe(depth_topic_compressed, 10, &SysCtrl::GetDepthCompressed_mynteye, p_sysctrl);
//    raw_depth_sub = nh.subscribe(depth_topic_raw, 10, &SysCtrl::GetDepthRaw_mynteye, p_sysctrl);

    prob_obstacle_sub = nh.subscribe(prob_obstacle_topic, 10, &SysCtrl::avoid_obstacle_callback_1,p_sysctrl);
    prob_obstacle_dev_sub = nh.subscribe(prob_obstacle_dev_topic, 10, &SysCtrl::GetDistance_callback, p_sysctrl);
    camera_pose_sub = nh.subscribe(pose_topic, 10, &SysCtrl::GetCameraPose_callback, p_sysctrl);
    temp_sub = nh.subscribe(temp_topic, 10, &SysCtrl::GetTemp_callback, p_sysctrl);

//    std::string imu_topic = "/mynteye/imu/data_raw";  //订阅imu
//    imu_sub = nh.subscribe(imu_topic, 10, &SysCtrl::GetIMU_callback, p_sysctrl);

    ros::spin();
	return 0;
}


