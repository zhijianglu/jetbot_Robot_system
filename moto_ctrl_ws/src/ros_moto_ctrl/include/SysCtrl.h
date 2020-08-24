//
// Created by lab on 2020/7/18.
//

#ifndef SYSCTRL_H
#define SYSCTRL_H
#include "ros/ros.h"
#include "sensor_msgs/image_encodings.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64MultiArray.h"
#include <image_transport/image_transport.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Dense>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <list>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <map>
#include <cmath>
#include <math.h>
#include <thread>
#include <mutex>
#include <stdio.h>
#include <queue>
#include "utils.h"
#include <map>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/thread/thread.hpp>
#include "Searcher.h"

using namespace message_filters;
using namespace std;
using namespace cv;
using namespace sensor_msgs;
using namespace Eigen;


class SysCtrl
{
public:
	SysCtrl(ros::NodeHandle& n);
	void readParameters(string config_file);
	~SysCtrl();

	//运动模式
	enum MovingMode{turn, tracking, avoid,warning};
    enum CamMode{search_bottle, look_obstacle, look_road};

    //瓶子检测
    Mat Bottle_img;
    Searcher* searcher;
    vector<double> perset_position;
    int search_wait_iter;
    int search_wait;


    double cam_turn_step = 0;  //相机转动为0~350
    int cam_direction = 1;  //

    bool searching_bottle = false;
    bool returning_voyage = false;
    bool start_yolo_pub = false;  //一开始的时候不启动yolo节点

    int nstep_turn_90deg_iter; //向右旋转90度所需次数
    int nstep_turn_90deg; //向右旋转90度所需次数

    int nstep_turn_90deg_back; //旋转90度返回所需次数
    int searching_direction = -1; //旋转方向  默认左转，这里是指定搜索水瓶的方向


    //定位相关
    //位姿
    Vector3d curr_t;
    Matrix3d curr_R;
    Vector3d last_t;
    Matrix3d last_R;
    double thes_angle = M_PI/30; //M_PI/2是90°，这里指的最大允许的偏角
    Vector3d ahead_direction;
    Vector3d left_direction;
    Vector3d right_direction;
    Vector3d start_position;

    bool init_pose = false;
    double latest_time;
    Eigen::Vector3d latest_P, latest_V, latest_Ba, latest_Bg, latest_acc_0, latest_gyr_0;
    Eigen::Quaterniond latest_Q;
    Vector3d g;

    //PID参数
    double e_k = 0;
    double e_k_1 = 0;
    double e_k_2 = 0;
    double Kp;
    double Ki;
    double Kd;
    double max_delta_u;
    double min_delta_u;

    Matrix3d ref_R;

    std::mutex m_buf;
    queue<double> sin_x;
    queue<double> sin_x_imu;

    int win_size_direction = 5;
    int win_size_imu = 200;
    double inte_sin_x = 0;
    int stright_cnt = 0;

    bool avoid_loop;
    double cam_avoid_iter =0;
    int body_avoid_iter = 0;
    int body_avoid_step = 30;
    int warning_step_back_iter = 0;
    int warning_step_turn_iter = 0;
    MovingMode moving_mode;
    MovingMode moving_mode_bak;
    CamMode cam_mode;
	double theta;  //正的就是右拐
    int obs_direction;
    int delay_frame;
    vector<double> init_position;

	//节点发布
	ros::Publisher goforward_pub;
	ros::Publisher gobackward_pub;
	ros::Publisher turnleft_pub;
	ros::Publisher turning_pub;
	ros::Publisher turnright_pub;
	ros::Publisher headup_pub;
	ros::Publisher headdown_pub;
	ros::Publisher cam_turn_pub;
    ros::Publisher prob_obstacle_pub;
    image_transport::Publisher img4yolo_pub;
    image_transport::Publisher sub_img_pub;


    //储存近win_size帧的障碍物概率
	list<double> prob_r;
	list<double> prob_l;

	double tol_prob_r; //窗口内的所有帧的障碍概率之和
	double tol_prob_l;
    bool cam_turning = false;

    //避障参数
	double curr_prob_r;
	double curr_prob_l;
	double curr_distance;
	double warning_distance = 15;
	double obstacle_distance = 40.0;  //初始障碍物的距离，保证系统可以正常启动即可
    queue<double> distance_buf;

    int obs_max = 255;  //双目避障距离,默认值
    int obs_min = 130;  //双目避障距离,默认值

	double distance_thes = 35.0;
	double win_size_distance = 3;  //延时10ms * win_size
    int win_size_depth_prob; //win_size 窗口大小
    double thres_val; //win_size 窗口大小 * 单帧阈值
    double trust = 60; //单帧认定为障碍物的像素数量

	//图像
    Mat current_left_img;
    Mat current_right_img;
    Mat current_depth;
    Mat current_est_depth;
    Mat current_depth_raw;

    //运动参数
    double stright_speed = 0.9;
    double stright_step = 0.01;
    double turn_speed = 1.2;
    double turn_step = 0.007;
    int n_turn_step = 0;


    const int imageWidth = 752; //摄像头的分辨率
    const int imageHeight = 480;
    Size imageSize = Size(imageWidth, imageHeight);

    Mat ObstacleMat;
    Mat nObstacleMat;

    Rect validROIL;//图像校正之后，会对图像进行裁剪，这里的validROI就是指裁剪之后的区域
    Rect validROIR;

    Mat mapLx, mapLy, mapRx, mapRy;     //映射表
    Mat Rl, Rr, Pl, Pr, Q;              //校正旋转矩阵R，投影矩阵P 重投影矩阵Q
    Mat xyz;              //三维坐标

    Point origin;         //鼠标按下的起始点
    Rect selection;      //定义矩形选框
    bool selectObject = true;    //是否选择对象

    int blockSize = 0, uniquenessRatio =0, numDisparities=0;
    Ptr<StereoBM> bm = StereoBM::create(16, 9);


//预先标定好的相机的参数，无需修改
    Mat cameraMatrixL = (Mat_<double>(3, 3) <<
        368.56741148924505, 0.0, 399.2271897860737,
        0.0, 368.6623501271576, 235.38396774097566,
        0.0, 0.0, 1.0);
    Mat distCoeffL = (Mat_<double>(5, 1) << -0.029529465378201628, 0.030563404238331246, -0.05229612619984657, 0.02533392726758464, 0.0);

    Mat cameraMatrixR = (Mat_<double>(3, 3) <<
                                            367.2116205882629, 0.0, 384.22852854185055,
        0.0, 367.30682474903455, 239.98742030833736,
        0.0, 0.0, 1.0);
    Mat distCoeffR = (Mat_<double>(5, 1) << -0.019693813428748954, -0.009211737175498618, 0.008919046579330396, -0.005076089900038799, 0.0);

//param 2
    Mat T = (Mat_<double>(3, 1) <<
        0.0008  ,
        -0.1216 ,
        0.0016  );//T平移向量
    Mat R = (Mat_<double>(3, 3) <<
        1.0000 , -0.0017 ,  0.0068 ,
        0.0019 ,  0.9999 , -0.0162 ,
        -0.0068,   0.0163,   0.9998
    );


    void stereo_match();

    //回调
    void GetDistance_callback(const std_msgs::Float32MultiArrayConstPtr &msg);
    void GetTemp_callback(const std_msgs::Float32MultiArrayConstPtr &msg);

    void GetCameraPose_callback(const nav_msgs::OdometryConstPtr &pose_msg);

    void ImageCallbackRaw(const sensor_msgs::ImageConstPtr& msg);
	void GetAndPubImg_jetcam(const sensor_msgs::CompressedImageConstPtr &msg);
    void GetImgCompressed_mynteye(const sensor_msgs::CompressedImageConstPtr &msg_img_left,const sensor_msgs::CompressedImageConstPtr &msg_img_right);
    void GetBottle_darknet(const sensor_msgs::ImageConstPtr &box_img, const std_msgs::Float64MultiArrayConstPtr &bottle_boxes);
    void avoid_obstacle_callback(const std_msgs::Float32MultiArrayConstPtr &msg);
    void avoid_obstacle_callback_1(const std_msgs::Float32MultiArrayConstPtr &prob_msg);
    void ResetCam();

    bool in_searching();
    bool avoid_warning();
    void GetDepthCompressed_mynteye(const sensor_msgs::CompressedImageConstPtr &depth_msg);
    void GetDepthRaw_mynteye(const sensor_msgs::ImageConstPtr &depth_msg);
    void cam_turn();
    void detectObs();
    void GetObsProb_callback(const std_msgs::Float32MultiArrayConstPtr &msg);
    void GetIMU_callback(const sensor_msgs::ImuConstPtr &imu_msg);
    void fastPredictIMU(double t, Eigen::Vector3d linear_acceleration, Eigen::Vector3d angular_velocity);

    void RemoveSmallRegion(cv::Mat& Src, cv::Mat& Dst, int AreaLimit, int CheckMode, int NeihborMode);
    void tracking_model();


};


#endif //SYSCTRL_H
