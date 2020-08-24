//
// Created by lab on 2020/7/28.
//

#ifndef SRC_SEARCHER_H
#define SRC_SEARCHER_H
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

#include <map>
#include <cmath>
#include <thread>
#include <mutex>
#include <cstdio>
#include <queue>
#include "utils.h"
#include <map>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/thread/thread.hpp>
// accumulate example
#include <iostream>     // std::cout
#include <functional>   // std::minus
#include <numeric>      // std::accumulate
#include <vector>      // std::accumulate
#include <cmath>      // std::accumulate


using namespace message_filters;
using namespace std;
using namespace cv;
using namespace sensor_msgs;
using namespace Eigen;

//对接yolo3的一个类
class Searcher
{
public:
    explicit Searcher(ros::NodeHandle& nh);
    void BottleDetect_loop();
    void ProcessBottle();

    void ResetDetection();  //单次检测中检测到了无效的水瓶，重新检测
    void ResetBottle();  //开始新一次的水瓶检测，并且将上一次的水瓶

    void GetBottleImg_callback(const sensor_msgs::ImageConstPtr &bottle_img);
    void GetBottleBox_callback(const std_msgs::Float64MultiArrayConstPtr &bottle_box);

    //输出
    string output_path;

    //订阅
    ros::Subscriber bottle_img_sub;
    ros::Subscriber bottle_box_sub;

    //发布水瓶
    Mat max_prob_bottle;
    double max_prob = 0;
    vector<cv::Mat> vBottle_results;
    image_transport::Publisher bottle_result_pub;
    std_msgs::Header curr_header;

    //bottle相关
    int search_time = -1;  //未被初始化,需要运行ResetBottle初始化
    int search_iter = 0;
    bool one_detech_finished = false;
    int water_line_win;  //前面若干帧中至少2帧是被判定为有效的水瓶线，才认定为有水
    vector<list<double>> bottle_water_prob;
    double line_rate_cnt = 0;
    bool have_water = false;
    double water_line_thres;

    Mat current_bottle_img;
    Rect2i curr_box;
    Rect2i last_box;
    double curr_prob;

    //buffer
    queue<sensor_msgs::ImageConstPtr> img_buf;
    queue<std_msgs::Float64MultiArrayConstPtr> box_buf;
    std::mutex m_buf;


};


#endif //SRC_SEARCHER_H
