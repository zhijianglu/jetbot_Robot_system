//
// Created by lab on 2020/7/20.
//

//
// Created by lab on 2020/7/17.
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


#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/thread/thread.hpp>
#include "sensor_msgs/image_encodings.h"

using namespace message_filters;
using namespace std;
using namespace cv;
using namespace sensor_msgs;

using  namespace  std;
using  namespace  cv;
int cnt = 0;

typedef message_filters::sync_policies::ApproximateTime<CompressedImage, CompressedImage> sync_policy_classification;

string path_to_save = "/home/lab/Project/stereo_data/data_0";

Mat current_left_img,current_right_img;
void GetImgCompressed_mynteye(const sensor_msgs::CompressedImageConstPtr &msg_img_left,const sensor_msgs::CompressedImageConstPtr &msg_img_right)
{
    cv_bridge::CvImagePtr cv_ptr_img_left = cv_bridge::toCvCopy(msg_img_left, sensor_msgs::image_encodings::MONO8);
    current_left_img = cv_ptr_img_left->image;

    cv_bridge::CvImagePtr cv_ptr_img_right = cv_bridge::toCvCopy(msg_img_right, sensor_msgs::image_encodings::MONO8);
    current_right_img = cv_ptr_img_right->image;
    Mat hcat;

    cv::hconcat(current_left_img,current_right_img,hcat);

    cv::imshow("left_and_right", hcat);

    cv::waitKey(1);
    stringstream ss_l,ss_r;
    ss_l <<path_to_save +"/image_0/"<< setw(8) << setfill('0')<<cnt<<".jpg";
    imwrite(ss_l.str(),current_left_img);

    ss_r <<path_to_save +"/image_1/"<< setw(8) << setfill('0')<<cnt<<".jpg";
    imwrite(ss_r.str(),current_right_img);
    cnt++;

}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "jetbot_ctrl");
    ros::NodeHandle nh;

    std::string leftimage_topic_compressed = "/mynteye/left_rect/image_rect/compressed";  //订阅
    std::string rightimage_topic_compressed = "/mynteye/right_rect/image_rect/compressed";  //订阅

    message_filters::Subscriber<CompressedImage> left(nh, leftimage_topic_compressed, 1);
    message_filters::Subscriber<CompressedImage> right(nh, rightimage_topic_compressed, 1);
    message_filters::Synchronizer<sync_policy_classification> sync(sync_policy_classification(100), left, right);
    sync.registerCallback(boost::bind(GetImgCompressed_mynteye,_1, _2));

    ros::spin();
    return 0;
}


