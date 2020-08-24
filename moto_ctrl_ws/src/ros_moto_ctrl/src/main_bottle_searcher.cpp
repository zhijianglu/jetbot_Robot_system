//
// Created by lab on 2020/7/17.
//

#include "ros/ros.h"
#include "sensor_msgs/image_encodings.h"
#include "Searcher.h"
#include <opencv2/opencv.hpp>

Searcher* p_searcher;

using namespace std_msgs;
typedef message_filters::sync_policies::ApproximateTime<CompressedImage, CompressedImage> sync_policy_classification;
image_transport::Publisher img4yolo_pub;

int pub_cnt = 0;
const int max_search = 40;
bool start_yolo_pub = false;
double cam_turn = 350;
int direction = 1;

double cam_turn_step = 10;

ros::Publisher cam_turn_pub;

void GetImgCompressed_mynteye(const sensor_msgs::CompressedImageConstPtr &msg_img_left,const sensor_msgs::CompressedImageConstPtr &msg_img_right)
{
    cv_bridge::CvImagePtr cv_ptr_img_left = cv_bridge::toCvCopy(msg_img_left, sensor_msgs::image_encodings::MONO8);
    Mat current_left_img = cv_ptr_img_left->image;

    imshow("current_left_img", current_left_img);
    int key = waitKey(1);

    if (!start_yolo_pub && key == 's')
    {   //重设
        p_searcher->ResetBottle();
        start_yolo_pub = true;
        pub_cnt = 0;
        cam_turn = 0;
    }


    //todo 发布点头消息
    std_msgs::Float64MultiArray ctl_head_msg;
    cam_turn += direction*5;
    if(cam_turn>=360){
        direction = -1;
    }else if(cam_turn<=0){
        direction = 1;
    }
    ctl_head_msg.data = std::vector<double>{cam_turn, cam_turn};
    cam_turn_pub.publish(ctl_head_msg);



    if(start_yolo_pub){
        pub_cnt++;

        sensor_msgs::ImagePtr sub_msg;
        sub_msg = cv_bridge::CvImage(msg_img_left->header, "mono8", current_left_img).toImageMsg();
        img4yolo_pub.publish(sub_msg);

        cout<<pub_cnt<<endl;

        //camera turn
//        std_msgs::Float64MultiArray ctl_msg;
//        cam_turn+=10;
//        if(cam_turn)
//        ctl_msg.data = std::vector<double>{0, 400};
//        cam_turn_pub.publish(ctl_msg);



        if(pub_cnt >= max_search && p_searcher->one_detech_finished == true)
            start_yolo_pub = false;
    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "jetbot_ctrl");
    ros::NodeHandle n;
//	p_sysctrl.reset(new SysCtrl(nh));
    image_transport::ImageTransport it(n);  //image_transport
    img4yolo_pub = it.advertise("/darknet/img_input", 100);
    p_searcher = new Searcher(n);

    cam_turn_pub = n.advertise<std_msgs::Float64MultiArray>("/jetbot_ctrl/camturn", 10);


    //read parameter
    string config_file = "/home/lab/Project/FullRobotSystem/moto_ctrl_ws/src/ros_moto_ctrl/config/config0.yaml";
    FILE *fh = fopen(config_file.c_str(), "r");
    if (fh == NULL)
    {
        ROS_WARN("config_file dosen't exist; wrong config_file path");
        ROS_BREAK();
        return -1;
    }
    fclose(fh);

    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if (!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    p_searcher->water_line_win = fsSettings["water_line_win"];
    p_searcher->water_line_thres = fsSettings["water_line_thres"];



    std::string leftimage_topic_compressed = "/mynteye/left_rect/image_rect/compressed";  //订阅图像
    std::string rightimage_topic_compressed = "/mynteye/right_rect/image_rect/compressed";  //订阅图像

    message_filters::Subscriber<CompressedImage> left(n, leftimage_topic_compressed, 1);
    message_filters::Subscriber<CompressedImage> right(n, rightimage_topic_compressed, 1);
    message_filters::Synchronizer<sync_policy_classification> sync(sync_policy_classification(100), left, right);
    sync.registerCallback(boost::bind(GetImgCompressed_mynteye,_1, _2));

    ros::spin();
    return 0;
}