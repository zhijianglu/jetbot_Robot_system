//
// Created by lab on 2020/7/28.
//

#include "Searcher.h"


Searcher::Searcher(ros::NodeHandle& nh){

    curr_box = Rect2i(0,0,0,0);
    last_box = curr_box;
    one_detech_finished = false;

    output_path = "/home/lab/Project/FullRobotSystem/output/";  //输出文件夹
    string bottle_img_topic = "/darknet/bottle_img";
    string bottle_box_topic = "/darknet/bottle_box";
    bottle_img_sub = nh.subscribe(bottle_img_topic, 10, &Searcher::GetBottleImg_callback, this);
    bottle_box_sub = nh.subscribe(bottle_box_topic, 10, &Searcher::GetBottleBox_callback, this);

    std::thread process_thread{&Searcher::BottleDetect_loop,this};
    process_thread.detach();

    cv::namedWindow("current_bottle_img",CV_WINDOW_NORMAL);  //显示图像
    cv::resizeWindow("current_bottle_img", 300, 600);

    image_transport::ImageTransport it(nh);  //image_transport
    bottle_result_pub = it.advertise("/bottle_result", 100);
}

void Searcher::GetBottleImg_callback(const sensor_msgs::ImageConstPtr &bottle_img){
    m_buf.lock();
    img_buf.push(bottle_img);
    m_buf.unlock();
}

void Searcher::GetBottleBox_callback(const std_msgs::Float64MultiArrayConstPtr &bottle_box){
    m_buf.lock();
    box_buf.push(bottle_box);
    m_buf.unlock();
}

void Searcher:: BottleDetect_loop()
{
    while (true)
    {
        m_buf.lock();

        if (img_buf.size() != 0 && box_buf.size() != 0)
        {
            //todo 获取图像
            cv_bridge::CvImagePtr
                cv_ptr_compressed = cv_bridge::toCvCopy(img_buf.back(), sensor_msgs::image_encodings::MONO8);

            current_bottle_img = cv_ptr_compressed->image;
            curr_header = img_buf.back()->header;
            img_buf.pop();
//            imshow("current_bottle_img", current_bottle_img);
//            waitKey(1);

            //todo 获取对应图像的box信息
            curr_box.x = box_buf.back()->data[0];
            curr_box.y = box_buf.back()->data[1];
            curr_box.width = box_buf.back()->data[2];
            curr_box.height = box_buf.back()->data[3];
            curr_prob = box_buf.back()->data[4];
            box_buf.pop();
            double curr_middle_dis = abs(476 - curr_box.x + curr_box.width / 2);  //376是图像中心
            double last_middle_dis = abs(476 - last_box.x + last_box.width / 2);
//            cout<<"--------------------"<<endl;
//            cout<<"curr_middle_dis:"<<curr_middle_dis<<endl;
//            cout<<"last_middle_dis:"<<last_middle_dis<<endl;

            if ( abs(curr_middle_dis - last_middle_dis) > 100 )  //前后两帧box相隔太远了，要判断哪个瓶子不是本次应该测的
            {
                if (last_middle_dis > curr_middle_dis)  //如果是本次的比较靠近图像中心
                {
                    ResetDetection(); //重来
                    last_box = curr_box;
                    cout << "result has been reset" << endl;
                }
            }else{
                ProcessBottle();
                last_box = curr_box;
            }

        }

        m_buf.unlock();

        //小憩一会儿
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

void Searcher::ProcessBottle()
{
    search_iter++;

    if (bottle_water_prob.size() < search_time + 1)
    {
        bottle_water_prob.push_back(list<double>{});
        vBottle_results.push_back(cv::Mat(cv::Size(80,80),CV_8SC3,Scalar(0,0,0)));
        line_rate_cnt = 0;
    }

    Mat processed_botle;
    processed_botle = current_bottle_img.clone();
    Canny(processed_botle, processed_botle, 50, 100,3);
//    imshow("current_bottle_img",processed_botle);
//    waitKey(1);

    int col_min = 0;
    int col_max = processed_botle.cols;

    int row_min = processed_botle.rows/5;
    int row_max = 4*processed_botle.rows/5;

    int target_line = row_min;
    int max_score = 0;
    int curr_score = 0;

    for (int r = row_min; r < row_max-3 ; r++){
        for (int c = col_min; c < col_max; c++)
        {
            if (processed_botle.at<uchar>(r, c) == 255 ||
                processed_botle.at<uchar>(r - 1, c) == 255 ||
                processed_botle.at<uchar>(r + 1, c) == 255  || processed_botle.at<uchar>(r + 1, c) == 255
                    )
            {
                curr_score += 1;
            }
        }

        if(curr_score>max_score){
            max_score = curr_score;
            target_line = r;
        }
        curr_score = 0;
    }

    double nline_history = bottle_water_prob[search_time].size();

    if (max_score > water_line_thres * (col_max - col_min)   )  //线长大于3/4瓶子图像宽度，认定为有水
    {
        double line_rate = (double) target_line / (double) processed_botle.rows;

        if (nline_history < water_line_win)  //在前面几帧直接就加入了
        {
            bottle_water_prob[search_time].push_back(line_rate);
            line_rate_cnt += line_rate;
        }
        else
        {
            if (abs((line_rate_cnt / water_line_win) - line_rate) < 0.2)  //如果本次与均值相差不是太远
            {
                bottle_water_prob[search_time].push_back(line_rate);
                line_rate_cnt = line_rate_cnt + line_rate - bottle_water_prob[search_time].front();
                bottle_water_prob[search_time].pop_front();
            }
        }
    }

    cv::cvtColor(current_bottle_img,current_bottle_img,CV_GRAY2BGR,0);

    if (search_iter >= water_line_win * 5)
    {
        one_detech_finished = true;
        //也就是说哦至少百分之60的图像是要被准确识别水平线的,否则将被认定为无水
        if ((search_iter == water_line_win * 5) && nline_history == water_line_win)
        {
            //只判断一次
            have_water = true;
        }

        if (have_water)
        {
            //说明确实是有水的
            cv::line(current_bottle_img,
                     Point2i(0, (line_rate_cnt / water_line_win) * processed_botle.rows),
                     Point2i(current_bottle_img.cols, (line_rate_cnt / water_line_win) * processed_botle.rows),
                     Scalar(0, 0, 255),
                     2);

            cv::putText(current_bottle_img,
                        to_string(search_time + 1),
                        Point2i(processed_botle.cols / 2, 8),
                        0.1,
                        0.3,
                        Scalar(0, 0, 255));

            imshow("current_bottle_img", current_bottle_img);

            if (curr_prob > max_prob)
            {

//                current_bottle_img.resize(current_bottle_img,Size(100,100),1,);
                Size re_size;
                re_size.height = 200;
                re_size.width = (200 / current_bottle_img.rows) * current_bottle_img.cols;

                cv::resize(current_bottle_img, current_bottle_img, re_size);

                vBottle_results[search_time] = current_bottle_img.clone();  //刷新图像

                Mat combined_bottle = current_bottle_img;

                if (search_time > 0)
                {  //两幅以上的图像
                    cv::hconcat(vBottle_results[0], vBottle_results[1], combined_bottle);

                    for (int i = 2; i < vBottle_results.size(); ++i)
                    {
                        cv::hconcat(combined_bottle, vBottle_results[i], combined_bottle);
                    }
                }

                sensor_msgs::ImagePtr bottle_results_msg;
                bottle_results_msg = cv_bridge::CvImage(curr_header, "bgr8", combined_bottle).toImageMsg();
                bottle_result_pub.publish(bottle_results_msg);
            }
            waitKey(1);
        }
        else
        {
            //说明确实是没有水的
            cv::line(current_bottle_img,
                     Point2i(0, processed_botle.rows),
                     Point2i(current_bottle_img.cols, processed_botle.rows),
                     Scalar(0, 255, 0),
                     2);

            cv::putText(current_bottle_img,
                        to_string(search_time + 1),
                        Point2i(processed_botle.cols / 2, 8),
                        0.1,
                        0.3,
                        Scalar(0, 255, 0));
            imshow("current_bottle_img", current_bottle_img);

            if(curr_prob>max_prob){
//                current_bottle_img.resize(current_bottle_img,Size(100,100),1,);
                Size re_size;
                re_size.height = 200;
                re_size.width = (200/current_bottle_img.rows)*current_bottle_img.cols;

                cv::resize(current_bottle_img, current_bottle_img, re_size);

                vBottle_results[search_time] = current_bottle_img.clone();  //刷新图像

                Mat combined_bottle = current_bottle_img;

                if(search_time>0){  //两幅以上的图像
                    cv::hconcat(vBottle_results[0],vBottle_results[1],combined_bottle);

                    for (int i = 2; i < vBottle_results.size(); ++i)
                    {
                        cv::hconcat(combined_bottle,vBottle_results[i],combined_bottle);
                    }
                }

                sensor_msgs::ImagePtr bottle_results_msg;
                bottle_results_msg = cv_bridge::CvImage(curr_header, "bgr8", combined_bottle).toImageMsg();
                bottle_result_pub.publish(bottle_results_msg);
            }

            waitKey(1);
        }
        imwrite(output_path + to_string(search_time+1) + ".jpg", current_bottle_img);
    }
    else
    {
        imshow("current_bottle_img", current_bottle_img);
        waitKey(1);
    }





//    cv::rectangle(current_bottle_img,Point2i(col_min, row_min), Point2i(col_max, row_max),3, 2,0);

//    Mat results;
//    cv::hconcat(current_bottle_img,processed_botle,results);
//    imshow("current_bottle_img",results);
//    waitKey(1);
}

void Searcher::ResetDetection(){
    search_iter = 0;
    line_rate_cnt = 0;
    have_water = false;
    one_detech_finished = false;

    if (bottle_water_prob.size() == search_time + 1)
        bottle_water_prob[search_time].clear();

}

void Searcher::ResetBottle(){  //开始新一次检测
    max_prob = 0;
    max_prob_bottle = cv::Mat(cv::Size(80,80),CV_8SC3,Scalar(0,0,0));
    search_time++;
    have_water = false;
    ResetDetection();
    curr_box = Rect2i(0,0,0,0);
    last_box = curr_box;
}





