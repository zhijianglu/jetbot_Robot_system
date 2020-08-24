//
// Created by lab on 2020/7/18.
//

#include "SysCtrl.h"

SysCtrl::SysCtrl(ros::NodeHandle& n)
{
    searcher = new Searcher(n);
    start_yolo_pub  = false;

    win_size_depth_prob = 2;
    thres_val = trust * win_size_depth_prob;
    prob_l = list<double>(win_size_depth_prob, 0);
    prob_r = list<double>(win_size_depth_prob, 0);

    tol_prob_r = 0;
    tol_prob_l = 0;

    moving_mode = tracking;
    avoid_loop = false;
    theta = 0;
    obs_direction = 0;
    delay_frame = 0;

    for (int i = 0; i < win_size_direction; ++i){
        sin_x.push(0.0); //初始角度为0
    }

    for (int i = 0; i < win_size_distance; ++i){
        distance_buf.push(40.0); //初始距离设置一个可以让系统启动第一步的距离，这时候可能还没有距离数据
    }


    gobackward_pub = n.advertise<std_msgs::Float64MultiArray>("/jetbot_ctrl/gobackward", 10);
    headup_pub =  n.advertise<std_msgs::Float64MultiArray>("/jetbot_ctrl/headup", 10);
    headdown_pub = n.advertise<std_msgs::Float64MultiArray>("/jetbot_ctrl/headdown", 10);
    turnleft_pub  =   n.advertise<std_msgs::Float64MultiArray>("/jetbot_ctrl/turnleft", 10);
    turnright_pub =  n.advertise<std_msgs::Float64MultiArray>("/jetbot_ctrl/turnright", 10);

    turning_pub =  n.advertise<std_msgs::Float64MultiArray>("/jetbot_ctrl/turning_angle", 10);
    goforward_pub =  n.advertise<std_msgs::Float64MultiArray>("/jetbot_ctrl/goforward", 10);
    cam_turn_pub = n.advertise<std_msgs::Float64MultiArray>("/jetbot_ctrl/camturn", 10);
    curr_distance = 10e10;
    init_position = vector<double>{0,0};

    ahead_direction << 1, 0, 0;
    init_pose = false;

//    std_msgs::Float64MultiArray ctl_msg;
//    ctl_msg.data = init_position;
//    cam_turn_pub.publish(ctl_msg);
    nObstacleMat = cv::Mat(imageSize,CV_8UC1,Scalar_<uchar>(0));  //开始像素数目为0
    ObstacleMat = cv::Mat(imageSize,CV_8UC1,Scalar_<uchar>(255));  //起初都是白色


//    ctl_msg.data = std::vector<double>{stright_step*2, stright_speed};
//    goforward_pub.publish(ctl_msg);

    cout<<"start generate control command====="<<endl;

    image_transport::ImageTransport it(n);  //image_transport
    sub_img_pub = it.advertise("/camera/rgb/sub_img", 100);
    img4yolo_pub = it.advertise("/darknet/img_input", 100);

    prob_obstacle_pub = n.advertise<std_msgs::Float32MultiArray>("/prob_blocked", 10);

    //------去畸变参数
    stereoRectify(cameraMatrixL, distCoeffL, cameraMatrixR, distCoeffR, imageSize, R, T, Rl, Rr, Pl, Pr, Q, CALIB_ZERO_DISPARITY,
                  0, imageSize, &validROIL, &validROIR);

    initUndistortRectifyMap(cameraMatrixL, distCoeffL, Rl, Pr, imageSize, CV_32FC1, mapLx, mapLy);
    initUndistortRectifyMap(cameraMatrixR, distCoeffR, Rr, Pr, imageSize, CV_32FC1, mapRx, mapRy);
//    namedWindow("disparity", CV_WINDOW_AUTOSIZE);

    //todo 使用imu定位相关
    g << 0.0, 0.0, 9.81007;
    latest_time = 0;
    latest_P.setZero();
    latest_V.setZero();
    latest_Ba.setZero();
    latest_Bg.setZero();
    latest_acc_0.setZero();
    latest_gyr_0.setZero();
    latest_Q.setIdentity();
}

void SysCtrl::readParameters(string config_file)
{
    FILE *fh = fopen(config_file.c_str(), "r");
    if (fh == NULL)
    {
        ROS_WARN("config_file dosen't exist; wrong config_file path");
        ROS_BREAK();
        return;
    }
    fclose(fh);

    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if (!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

//----------------
    warning_distance = fsSettings["warning_distance"];
    nstep_turn_90deg = fsSettings["nstep_turn_90deg"];

//    nstep_turn_90deg_iter = fsSettings["nstep_turn_90deg_iter"];
//    nstep_turn_90deg_back = fsSettings["nstep_turn_90deg_back"];

    searching_direction = fsSettings["searching_direction"];
    thes_angle = fsSettings["thes_angle"];
    win_size_direction = fsSettings["win_size_direction"];
    distance_thes = fsSettings["distance_thes"];
    body_avoid_step = fsSettings["body_avoid_step"];

    Kp = fsSettings["Kp"];
    Ki = fsSettings["Ki"];
    Kd = fsSettings["Kd"];

    max_delta_u = fsSettings["max_delta_u"];
    min_delta_u = fsSettings["min_delta_u"];
    search_wait = fsSettings["search_wait"];

    searcher->water_line_win = fsSettings["water_line_win"];
    searcher->water_line_thres = fsSettings["water_line_thres"];

    double bottle_1 = fsSettings["bottle_1"];
    double bottle_2 = fsSettings["bottle_2"];
    double bottle_3 = fsSettings["bottle_3"];
    perset_position = vector<double>{bottle_1, bottle_2, bottle_3};

    cout << "preset prameters:-----------------------------"
         << "search_wait:" << search_wait << "\n"
         << "warning_distance:" << warning_distance << "\n"
         << "body_avoid_step:" << body_avoid_step << "\n"
         << "nstep_turn_90deg:" << nstep_turn_90deg << "\n"
         << "nstep_turn_90deg_back:" << nstep_turn_90deg_back << "\n"
         << "searching_direction:" << searching_direction << "\n"
         << "thes_angle:" << thes_angle << "\n"
         << "win_size_direction:" << win_size_direction << "\n"
         << "distance_thes:" << distance_thes << "\n"
         << "obs_min:" << obs_min << "\n"
         << "Kp:" << Kp << "\n"
         << "Ki:" << Ki << "\n"
         << "Kd:" << Kd << "\n"
         << "max_delta_u:" << max_delta_u << "\n"
         << "min_delta_u:" << min_delta_u << "\n"
         << "bottle_1:" << bottle_1 << "\n"
         << "bottle_2:" << bottle_2 << "\n"
         << "bottle_3:" << bottle_3 << "\n"
         << "water_line_win:" << searcher->water_line_win << "\n"
         << "water_line_thres:" << searcher->water_line_thres << "\n" << endl;

}


//todo---计算深度：----------------------------------------------------------
void SysCtrl::GetImgCompressed_mynteye(const sensor_msgs::CompressedImageConstPtr &msg_img_left,const sensor_msgs::CompressedImageConstPtr &msg_img_right)
{


    cv_bridge::CvImagePtr cv_ptr_img_left = cv_bridge::toCvCopy(msg_img_left, sensor_msgs::image_encodings::MONO8);
    current_left_img = cv_ptr_img_left->image;

    cv_bridge::CvImagePtr cv_ptr_img_right = cv_bridge::toCvCopy(msg_img_right, sensor_msgs::image_encodings::MONO8);
    current_right_img = cv_ptr_img_right->image;
    Mat hcat;

    cv::hconcat(current_left_img,current_right_img,hcat);

    cv::imshow("left_and_right", hcat);
    int key = cv::waitKey(1);

    if (key == 'q')
        ros::shutdown();
    else if(key == 's'){
        imwrite("/home/lab/Project/RobotPrj/output/imgs/stereo_left.png",current_left_img);
        imwrite("/home/lab/Project/RobotPrj/output/imgs/stereo_right.png",current_right_img);
        imwrite("/home/lab/Project/RobotPrj/output/imgs/gt_disparity.png",current_depth);
    }


    if(start_yolo_pub){
//    if(true){
        sensor_msgs::ImagePtr sub_msg;
        sub_msg = cv_bridge::CvImage(msg_img_left->header, "mono8", current_left_img).toImageMsg();
        img4yolo_pub.publish(sub_msg);
        //同时要上下摆动相机提高搜索效率

        std_msgs::Float64MultiArray ctl_head_msg;
        cam_turn_step += cam_direction*5;

        if(cam_turn_step>=360){  //到了最上面，向下回转
            cam_direction = -1;
        }else if(cam_turn_step<=130){
            cam_direction = 1;  //到了最下面的点，向上回转
        }
        ctl_head_msg.data = std::vector<double>{cam_turn_step, cam_turn_step};
        cam_turn_pub.publish(ctl_head_msg);
    }

    //todo generate depth---------------------------------

//    cvtColor(current_left_img, grayImageL, CV_BGR2GRAY);
//    cvtColor(current_right_img, grayImageR, CV_BGR2GRAY);

//    remap(grayImageL, rectifyImageL, mapLx, mapLy, INTER_LINEAR);
//    remap(grayImageR, rectifyImageR, mapRx, mapRy, INTER_LINEAR);

    blockSize = 5;
    uniquenessRatio = 7;
    numDisparities = 10;

    stereo_match();

}

void SysCtrl::stereo_match()
{
    //第一步生成深度图：
    bm->setBlockSize(2*blockSize+5);     //SAD窗口大小，5~21之间为宜
    bm->setROI1(validROIL);
    bm->setROI2(validROIR);
    bm->setPreFilterCap(30);
    bm->setMinDisparity(1);  //最小视差，默认值为0, 可以是负值，int型
    bm->setNumDisparities(numDisparities*16+16);//视差窗口，即最大视差值与最小视差值之差,窗口大小必须是16的整数倍，int型
    bm->setTextureThreshold(10);
    bm->setUniquenessRatio(uniquenessRatio);//uniquenessRatio主要可以防止误匹配
    bm->setSpeckleWindowSize(100);
    bm->setSpeckleRange(50);
    bm->setDisp12MaxDiff(-1);
    Mat disp, disp8;
    bm->compute(current_left_img, current_right_img, disp);//输入图像必须为灰度图
//    imshow("disp",disp);
    disp.convertTo(disp8, CV_8U, 255 / ((numDisparities * 16 + 16)*16.));//计算出的视差是CV_16S格式
    reprojectImageTo3D(disp, xyz, Q, true); //在实际求距离时，ReprojectTo3D出来的X / W, Y / W, Z / W都要乘以16(也就是W除以16)，才能得到正确的三维坐标信息。
    xyz = xyz * 16;
    double min_est;
    double max_est;
    int minIdx;
    int maxIdx;
    current_est_depth = disp8.clone();
    cv::minMaxIdx(disp8, &min_est, &max_est, &minIdx, &maxIdx);
    cv::applyColorMap((255.0 / (max_est)) * disp8, disp8, cv::COLORMAP_OCEAN);//转换成颜色编码的深度图
//    imshow("disparity", disp8);

    //第二步：去除离群点，深度截断判定障碍物
    detectObs();
}

void SysCtrl::detectObs(){

    //提取出距离比较近的一些点作为预警
    //设置最大和最小障碍物深度



    //obs pixel
    int obs_prob_l =0;
    int obs_prob_r =0;
    //点的历史
    int nObs_pixel = 0;
    int nhistory_obs = 1;  //由于滤除偶尔判断失误的点，深度连续超过1帧稳定才算数

    int col_min = imageWidth/2;
    int col_max = imageWidth;

    for (int c = col_min; c < col_max; c++)
    {
        for (int r = imageHeight/3; r < 4*imageHeight/5; r++)
        {
            int _depth = (int)current_est_depth.at<uchar>(r, c);
            uchar &nObs = nObstacleMat.at<uchar>(r, c);  //引用
            uchar &val_Obs = ObstacleMat.at<uchar>(r, c);

            if ( _depth <= obs_max && _depth >= obs_min )  //该像素小于阈值
            {
                if( nObs > nhistory_obs ){  //该像素连续nhistory_obs帧小于阈值,可信！
                    val_Obs = _depth;

                    if ( c < 3*imageWidth/4 )
                        obs_prob_l += (int) _depth;
                    else
                        obs_prob_r += (int) _depth;

                    nObs_pixel++;

                }else{
                    nObs = uchar(int(nObs) + 1);
                }
            }
            else{
                val_Obs = 255;
                nObs = nObs > 0 ? nObs-1:0;
            }
        }
    }

    RemoveSmallRegion(ObstacleMat, ObstacleMat,250, 1, 1);
    RemoveSmallRegion(ObstacleMat, ObstacleMat,400, 0, 0);

    obs_prob_l/=10000;
    obs_prob_r/=10000;
    cv::line(ObstacleMat,cv::Point2i(imageWidth/2,0),cv::Point2i(imageWidth/2,imageHeight),1,1);
    cv::line(ObstacleMat,cv::Point2i(3*imageWidth/4,0),cv::Point2i(3*imageWidth/4,imageHeight),1,1);

    cv::putText(ObstacleMat, "left:" + to_string(obs_prob_l), cv::Point2i(imageWidth / 2+30, 30), 1, 2, 1, 2);
    cv::putText(ObstacleMat, "right:" + to_string(obs_prob_r), cv::Point2i(3 * imageWidth / 4 + 30, 30), 1, 2, 1, 2);
    cv::putText(ObstacleMat, "distance:" + to_string(curr_distance).substr(0, 5), cv::Point2i((3 * imageWidth / 4) -90 , 90), 1, 1.5, 1, 2);

    imshow("obstacle_mat", ObstacleMat.colRange(imageWidth/2,imageWidth));
    std_msgs::Float32MultiArray prob_msg;

    float prob_l = obs_prob_l < 90 ? prob_l / 90 : 1;
    float prob_r = obs_prob_r < 90 ? prob_r / 90 : 1;

    prob_msg.data = std::vector<float>{prob_l,prob_r,(float)obs_prob_l,(float)obs_prob_r};
//    prob_msg.data = std::vector<float>{0, 0};

    prob_obstacle_pub.publish(prob_msg);
}

void SysCtrl::RemoveSmallRegion(cv::Mat& Src, cv::Mat& Dst, int AreaLimit, int CheckMode, int NeihborMode)
{
    int RemoveCount=0;       //记录除去的个数
    //记录每个像素点检验状态的标签，0代表未检查，1代表正在检查,2代表检查不合格（需要反转颜色），3代表检查合格或不需检查
    cv::Mat Pointlabel = cv::Mat::zeros( Src.size(), CV_8UC1 );

    if(CheckMode==1)
    {
//        std::cout<<"Mode: remove outlier. ";
        for(int i = 0; i < Src.rows; ++i)
        {
            uchar* iData = Src.ptr<uchar>(i);
            uchar* iLabel = Pointlabel.ptr<uchar>(i);
            for(int j = 0; j < Src.cols; ++j)
            {
                if (iData[j] < 10)
                {
                    iLabel[j] = 3;
                }
            }
        }
    }
    else
    {
//        cout<<"Mode: remove hole. ";
        for(int i = 0; i < Src.rows; ++i)
        {
            uchar* iData = Src.ptr<uchar>(i);
            uchar* iLabel = Pointlabel.ptr<uchar>(i);
            for(int j = 0; j < Src.cols; ++j)
            {
                if (iData[j] > 10)
                {
                    iLabel[j] = 3;
                }
            }
        }
    }

    vector<Point2i> NeihborPos;  //记录邻域点位置
    NeihborPos.push_back(Point2i(-1, 0));
    NeihborPos.push_back(Point2i(1, 0));
    NeihborPos.push_back(Point2i(0, -1));
    NeihborPos.push_back(Point2i(0, 1));
    if (NeihborMode==1)
    {
//		cout<<"Neighbor mode: 8邻域."<<endl;
        NeihborPos.push_back(Point2i(-1, -1));
        NeihborPos.push_back(Point2i(-1, 1));
        NeihborPos.push_back(Point2i(1, -1));
        NeihborPos.push_back(Point2i(1, 1));
    }
//	else cout<<"Neighbor mode: 4邻域."<<endl;
    int NeihborCount=4+4*NeihborMode;
    int CurrX=0, CurrY=0;
    //开始检测
    for(int i = 0; i < Src.rows; ++i)
    {
        uchar* iLabel = Pointlabel.ptr<uchar>(i);
        for(int j = 0; j < Src.cols; ++j)
        {
            if (iLabel[j] == 0)
            {
                //********开始该点处的检查**********
                vector<Point2i> GrowBuffer;                                      //堆栈，用于存储生长点
                GrowBuffer.push_back( Point2i(j, i) );
                Pointlabel.at<uchar>(i, j)=1;
                int CheckResult=0;                                               //用于判断结果（是否超出大小），0为未超出，1为超出

                for ( int z=0; z<GrowBuffer.size(); z++ )
                {

                    for (int q=0; q<NeihborCount; q++)                                      //检查四个邻域点
                    {
                        CurrX=GrowBuffer.at(z).x+NeihborPos.at(q).x;
                        CurrY=GrowBuffer.at(z).y+NeihborPos.at(q).y;
                        if (CurrX>=0&&CurrX<Src.cols&&CurrY>=0&&CurrY<Src.rows)  //防止越界
                        {
                            if ( Pointlabel.at<uchar>(CurrY, CurrX)==0 )
                            {
                                GrowBuffer.push_back( Point2i(CurrX, CurrY) );  //邻域点加入buffer
                                Pointlabel.at<uchar>(CurrY, CurrX)=1;           //更新邻域点的检查标签，避免重复检查
                            }
                        }
                    }

                }
                if (GrowBuffer.size()>AreaLimit) CheckResult=2;                 //判断结果（是否超出限定的大小），1为未超出，2为超出
                else {CheckResult=1;   RemoveCount++;}
                for (int z=0; z<GrowBuffer.size(); z++)                         //更新Label记录
                {
                    CurrX=GrowBuffer.at(z).x;
                    CurrY=GrowBuffer.at(z).y;
                    Pointlabel.at<uchar>(CurrY, CurrX) += CheckResult;
                }
                //********结束该点处的检查**********


            }
        }
    }

    CheckMode=255*(1-CheckMode);
    //开始反转面积过小的区域
    for(int i = 0; i < Src.rows; ++i)
    {
        uchar* iData = Src.ptr<uchar>(i);
        uchar* iDstData = Dst.ptr<uchar>(i);
        uchar* iLabel = Pointlabel.ptr<uchar>(i);
        for(int j = 0; j < Src.cols; ++j)
        {
            if (iLabel[j] == 2)
            {
                iDstData[j] = CheckMode;
            }
            else if(iLabel[j] == 3)
            {
                iDstData[j] = iData[j];
            }
        }
    }
//	cout<<RemoveCount<<" objects removed."<<endl;
}

//todo---获取距离：----------------------------------------------------------
void SysCtrl::GetDistance_callback(const std_msgs::Float32MultiArrayConstPtr &msg){

    if (msg->data[0] < 80 && msg->data[0] > 3)  //单位cm，超出范围为错误数据
    {
        curr_distance = msg->data[0];
        distance_buf.push(curr_distance);  //最新的数据放在back

        if (distance_buf.size() > win_size_distance)
        {
            distance_buf.pop();  //将最先放进去的剔除，front的数据删掉
        }

    }

}

//todo---获取温度：----------------------------------------------------------
void SysCtrl::GetTemp_callback(const std_msgs::Float32MultiArrayConstPtr &msg){

    cout << "current temperature is : [" << msg->data[0] << "] ℃" << endl;

}

//todo---判定障碍物：----------------------------------------------------------
void SysCtrl::GetObsProb_callback(const std_msgs::Float32MultiArrayConstPtr &msg){

    curr_prob_r = msg->data[2];
    curr_prob_l = msg->data[3];

    prob_l.push_back(curr_prob_l);
    prob_r.push_back(curr_prob_r);

    tol_prob_r = tol_prob_r + prob_r.back() - prob_r.front();  //加上最新的减去最旧的
    tol_prob_l = tol_prob_l + prob_l.back() - prob_l.front();  //加上最新的减去最旧的

    prob_r.pop_front();  //丢掉最旧的
    prob_l.pop_front();  //丢掉最旧的

//    cout<<"---------------"<<endl;
//    cout<<"curr_distance:"<<curr_distance<<endl;
//    cout<<"obstacle_distance:"<<obstacle_distance<<endl;

    if (obstacle_distance > curr_distance)
        obstacle_distance = curr_distance;  //当前检测到的障碍物的距离

    if(moving_mode==turn){
        if (obs_direction > 0)  //之前判定障碍物在右侧
        {
            //todo 这里的代码决定了：发现障碍物之后到底应该转多少角度再直走才能避免相撞
            obs_direction =
                (tol_prob_r < thres_val * 0.20) ||  //只有完全转到一个安全的方向才能允许继续前行
                ((curr_distance - obstacle_distance > 15) && (curr_distance < 400)) ? 0 : obs_direction;
                //因为转动，当前的距离相比于障碍物的距离增大了15cm，认定已经撇开障碍物，将方向设定为0

            //我们要求障碍物在右侧的深度值小于阈值的一定比例，因为这个过程小车还在往前走，很可能已经到了盲区
            return;
        }else if(obs_direction < 0){

//            cout << "tol_prob_l :" << tol_prob_l << "  thres_val * 0.35: " << thres_val * 0.35 << endl;

            obs_direction =
                (tol_prob_l < thres_val * 0.20) ||
                    ((curr_distance - obstacle_distance > 15) && (curr_distance < 400)) ? 0 : obs_direction;
            return;
        }

//        if (obs_direction > 0){  //之前判定障碍物在右侧
//            obs_direction = (curr_distance > distance_thes * 1.5) ? 0 : obs_direction;
//            return;
//        }
    }

    if (
        tol_prob_r > thres_val ||
        tol_prob_l > thres_val ||
        tol_prob_l+tol_prob_r > thres_val ||
        curr_distance < distance_thes)
    {

//        cout << "----------------------" << endl;
//        cout << "     should turn now: " << endl;
//
//        cout << "          tol_prob_r: " << tol_prob_r << endl;
//        cout << "          tol_prob_l: " << tol_prob_l << endl;
//        cout << "  distance_thes: " << distance_thes << endl;

//    if(  curr_distance < 20.0  )
        obs_direction = tol_prob_r > tol_prob_l ? 1 : -1;

    }
    else
    {
        obs_direction = 0;
    }

    obstacle_distance = curr_distance;  //刷新当前障碍物距离

}

//todo---判定障碍物失败后的预警
bool SysCtrl::avoid_warning(){
//  当双目没有成功避障，距离探测器检测到距离太小，为防止撞上去的最后一道防线
    std_msgs::Float64MultiArray ctl_msg;
    if(moving_mode!=warning && curr_distance<warning_distance){
        moving_mode_bak = moving_mode;
        moving_mode=warning;
        warning_step_back_iter = 15;
    }
    if(moving_mode==warning ){
        if(warning_step_back_iter>0){
            ctl_msg.data = std::vector<double>{stright_step*(0.45), stright_speed};
            gobackward_pub.publish(ctl_msg);
            warning_step_back_iter--;
            return false;
        }else{

            if(curr_distance<40) //搜索一个可以走的方向,只有找到大于40cm的方向才允许继续前进
            {
//                theta += direction*turn_step;  //旋转角按步长叠加
                ctl_msg.data = std::vector<double>{0.35*turn_step, turn_speed};
//                cout<<"direction*turn_step = "<<turn_step<<endl;
                turning_pub.publish(ctl_msg);
            }else{
                moving_mode = moving_mode_bak;
            }
        }
    }
    return true;
}

//todo---获取跟踪位姿：----------------------------------------------------------
void SysCtrl::GetCameraPose_callback(const nav_msgs::OdometryConstPtr &pose_msg)
{

    curr_t.x() = pose_msg->pose.pose.position.x;
    curr_t.y() = pose_msg->pose.pose.position.y;
    curr_t.z() = pose_msg->pose.pose.position.z;
//    cout << "current position: " << curr_t.transpose() << endl;

    Quaterniond q;
    q.x() = pose_msg->pose.pose.orientation.x;
    q.y() = pose_msg->pose.pose.orientation.y;
    q.z() = pose_msg->pose.pose.orientation.z;
    q.w() = pose_msg->pose.pose.orientation.w;
    curr_R = q.toRotationMatrix();
//    Vector3d  ref_x;
//    ref_x<<1,0,0;

    if(!init_pose){
        ahead_direction = curr_R.col(2);  // curr_R.col(2)是与跟踪角度  x轴
        left_direction = -curr_R.col(0);  // -curr_R.col(0)是与跟踪角度90°的左边  y轴
        right_direction = curr_R.col(0);  //  curr_R.col(0)是与跟踪角度90°的右边  -y轴
                                          //  curr_R.col(1)是与跟踪角度90°的上面， z轴
        start_position = curr_t;

        init_pose = true;
    }


    double angle = ahead_direction.dot(curr_R.col(0)) / curr_R.col(0).norm();
//    cout << "current angle: " << angle << endl;
//    cout << "    ahead_direction: " << ahead_direction << endl;
//    cout << "      curr_R.col(1): " << curr_R.col(1) << endl;

    m_buf.lock();

    sin_x.push(angle);
    inte_sin_x = inte_sin_x+sin_x.back()-sin_x.front();
    sin_x.pop();
    m_buf.unlock();
}

void SysCtrl::tracking_model(){
    m_buf.lock();
    //方法一：采用vins定位
    double curr_sin_x = sin_x.back();
    double mean_sin_x = inte_sin_x/win_size_direction;
//

    //方法二：采用imu预积分定位
//    double curr_sin_x = sin_x_imu.back();  //最新一帧的位姿放在back
    m_buf.unlock();

//    cout << "curr_sin_x: " << curr_sin_x << endl;
//    cout << "mean_sin_x: " << mean_sin_x << endl;
//    cout << "-----------"  << endl;


    std_msgs::Float64MultiArray ctl_msg;

//    cout<<theshold<<endl;

//方法一：比例调节
//    if (abs(curr_sin_x) > theshold && stright_cnt % 4 == 0)
//    {
//        double ctrl_step = turn_step * curr_sin_x;
//        ctl_msg.data = std::vector<double>{ctrl_step, turn_speed};
//        turning_pub.publish(ctl_msg);
//        stright_cnt++;
//
//    }
//    else
//    {
//        ctl_msg.data = std::vector<double>{stright_step, stright_speed};
//        goforward_pub.publish(ctl_msg);
//        stright_cnt++;
//    }

//方法二：PID调节

//    delay_frame = 1;
    if (abs(curr_sin_x) > sin(thes_angle) )//偏离了预先轨道，通过pid调节回来
    {

        e_k_2 = e_k_1;
        e_k_1 = e_k;
        e_k = curr_sin_x;
        double delta_u =
            Kp * (e_k - e_k_1) +
                    Ki * e_k +
                    Kd * (e_k - 2 * e_k_1 + e_k_2);

        if(abs(delta_u)<min_delta_u){
            delta_u = delta_u > 0 ? min_delta_u : -min_delta_u;
        }

        if(abs(delta_u)>max_delta_u){
            delta_u = delta_u > 0 ? max_delta_u : -max_delta_u;
        }

//        cout<<"ctrl delta_u: "<<delta_u<<endl;
//        double ctrl_step = turn_step * curr_sin_x;

        ctl_msg.data = std::vector<double>{ delta_u * turn_step, turn_speed};
        turning_pub.publish(ctl_msg);
    }
    else
    {
        //重设参数
        e_k = 0;
        e_k_1 = 0;
        e_k_2 = 0;

        ctl_msg.data = std::vector<double>{stright_step, stright_speed};
        goforward_pub.publish(ctl_msg);
    }
}

bool SysCtrl::in_searching()
{
    if (!searching_bottle && perset_position.size() != 0)  //当前不在搜索状态，那就进行判定是否需要进入搜索状态
    {
        //当前到达既定点，且不在避障状态，也不在回航状态，那就开始搜索，初始化搜索的所有值，包括搜索转动幅度之类的
        if ((curr_t - start_position).norm() > perset_position.front() && moving_mode == tracking
            && abs(sin_x.back()) < sin(thes_angle))
        {

            searching_bottle = true;
            search_wait_iter = search_wait;  //转过去等待search_wait帧搜索
            nstep_turn_90deg_iter = searching_direction > 0 ? nstep_turn_90deg : nstep_turn_90deg+2; //旋转90度所需次数
            nstep_turn_90deg_back = searching_direction > 0 ? nstep_turn_90deg+2 : nstep_turn_90deg; //旋转90度所需次数

            //相机上下摆动搜索瓶子
            cam_direction = 1;
            cam_turn_step = 0;

            return true;  // 返回true，表示处于搜索状态，跳过后面的所有电机控制指令
        }
        else
        {
            return false;  //正常情况下行驶，直接返回，执行主循环后面的控制代码
        }
    }

    if (searching_bottle && !returning_voyage)  //如果在搜索状态且不在回航状态
    {
//        double sin_angle = ahead_direction.dot(curr_R.col(0)) / curr_R.col(0).norm();
        //首先是计算当前偏差
        double sin_angle;
        std_msgs::Float64MultiArray ctl_msg;

        if(searching_direction<0){ //左转
            //
            sin_angle = left_direction.dot(curr_R.col(0)) / curr_R.col(0).norm();

        }else{
            sin_angle = right_direction.dot(curr_R.col(0)) / curr_R.col(0).norm();
        }

        bool shoule_pid_control = abs(sin_angle) > sin(thes_angle);




        if (nstep_turn_90deg_iter != 0 || shoule_pid_control )
        {
            if(nstep_turn_90deg_iter != 0)  //先手动转过去固定步伐
            {
                ctl_msg.data = std::vector<double>{searching_direction * turn_step, turn_speed};
                turning_pub.publish(ctl_msg);
                nstep_turn_90deg_iter--;
                return true;    // 如果是处于搜索状态，返回是，并且跳过后面的所有电机控制指令
            }
            //转得不够精确，再通过pid 控制微调
            if (shoule_pid_control)//偏离了预先轨道，通过pid调节回来
            {
                e_k_2 = e_k_1;
                e_k_1 = e_k;
                e_k = sin_angle;
                double delta_u =
                    Kp * (e_k - e_k_1) +
                        Ki * e_k +
                        Kd * (e_k - 2 * e_k_1 + e_k_2);

                if (abs(delta_u) < min_delta_u)
                {
                    delta_u = delta_u > 0 ? min_delta_u : -min_delta_u;
                }

                if (abs(delta_u) > max_delta_u)
                {
                    delta_u = delta_u > 0 ? max_delta_u : -max_delta_u;
                }

//        cout<<"ctrl delta_u: "<<delta_u<<endl;
//        double ctrl_step = turn_step * curr_sin_x;

                ctl_msg.data = std::vector<double>{delta_u * turn_step, turn_speed};
                turning_pub.publish(ctl_msg);
                return true;
            }
        }
        else if (search_wait_iter > 0 ) //转完之后还要等待40帧
        {
            if(search_wait_iter == search_wait)
            {
                searcher->ResetBottle(); //检测新的水瓶，并且保存上一次的水瓶
                start_yolo_pub = true;  //启动将图像发布到yolo输入节点
            }

            search_wait_iter--;

            return true;
        }
        else
        {
            if( !searcher->one_detech_finished ) return true;
            start_yolo_pub = false; // 关闭yolo节点，节省资源
            returning_voyage = true;
        }
    }
    else  //准备回航状态
    {
        if (nstep_turn_90deg_back > 0)
        { //开始回转
            std_msgs::Float64MultiArray ctl_msg;
            ctl_msg.data = std::vector<double>{-searching_direction * turn_step, turn_speed};
            turning_pub.publish(ctl_msg);
            nstep_turn_90deg_back--;
            return true;    // 如果是处于搜索状态，返回是，并且跳过后面的所有电机控制指令
        }
        else  //回转之后，重设所有值
        {
            if (perset_position.size() != 0)
                perset_position.erase(perset_position.begin());//删除第一个位置，因为该点已经完成搜索

            returning_voyage = false; //完成回转
            searching_bottle = false;
            ResetCam();
            return false; //继续执行后面的代码
        }
    }
}

void SysCtrl::ResetCam(){
    std_msgs::Float64MultiArray ctl_head_msg;
    ctl_head_msg.data = std::vector<double>{0, 0};
    cam_turn_pub.publish(ctl_head_msg);
}

//todo---避障主循环************************************************************************
void SysCtrl::avoid_obstacle_callback_1(const std_msgs::Float32MultiArrayConstPtr &prob_msg)
{
    //先判定当前是否属于搜索状态
    if (in_searching())
        return;

    //待发布的控制信息
    std_msgs::Float64MultiArray ctl_msg;

    //获取当前障碍物信息
    GetObsProb_callback(prob_msg);

    //如果当前距离太过近，到了双目测距盲区，开启预警模式
    if (!avoid_warning())
        return;

    if (delay_frame != 0)
    {
        delay_frame--;
        ctl_msg.data = std::vector<double>{stright_step * (0.45), stright_speed};
        goforward_pub.publish(ctl_msg);
        return;
    }

    if (!avoid_loop)
    {  //如果当前状态不是摆脱障碍物的状态

        if (obs_direction != 0)  //如果当前遇到障碍物
        {
            if (moving_mode == tracking)
            {  //恰好之前一次是没有障碍
                moving_mode = turn;  //将当前状态改为旋转模式
                theta = 0;  //重设当前旋转角
            }

            if (moving_mode == turn)
            {
//                cout << "**************** turning ****************" << endl;
                theta += obs_direction * 0.35 * turn_step;  //旋转角按步长叠加
                n_turn_step++;
                ctl_msg.data = std::vector<double>{obs_direction * 0.35*turn_step, turn_speed};
//                cout<<"direction*turn_step = "<<direction*turn_step<<endl;
                turning_pub.publish(ctl_msg);
            }
        }
        else
        {  //当前显示没有障碍物
            if (moving_mode == turn)
            {  //如果之前一直是旋转状态，步长在叠加状态
                avoid_loop = true;  //开启避障模式，直到摆脱障碍物
                body_avoid_iter = body_avoid_step;

                n_turn_step = min(n_turn_step,30); //最多60 下回转，要不然转360度就没有必要了
                if(theta<0)
                {
                    n_turn_step *= 0.2;  //如果是向右边回转的话，阻尼太大，需要将步数减小
                    cout<<"theta:"<<theta<<endl;
                }

//                delay_frame = 3;  //跳过接下来的帧，为相机转过方向延时
//                cam_mode = look_obstacle;
//                delay_frame = 15;
//                cam_turn();  //将相机转向障碍物方向
            }
            if (moving_mode == tracking)
            {  //如果当前状态是直走，无障碍物，那么继续直走
//                cout << "---------------- tracking ----------------" << endl;
                tracking_model();
            }
        }
    }
    else
    { //摆脱障碍物的状态，继续往前走，直到两侧都没有障碍物,再转回去
//        cout << "################ avoiding loop ################" << endl;

        if (body_avoid_iter > 0)
        {
            //先走 body_avoid_iter 步
            ctl_msg.data = std::vector<double>{stright_step, stright_speed};
            goforward_pub.publish(ctl_msg);
            body_avoid_iter--;
        }
        else
        {  //障碍摆脱，重设当前状态，并且将机器人转回原先方向

            //方法一：一次性转过去
//            //一步走theta角是回不去的，单步回去必须有一个倍数相乘
//            double scale_multi2one = 1.4;
//            double OneStepGoBack = theta * scale_multi2one;
//            int searching_direction = theta < 0 ? 1 : -1;
//
//            //左右轮是有阻尼的，需要进行差速调节
////            double left2right_damp = (56.0 / 51.0);
////
////            if (theta < 0)
////            {
////                OneStepGoBack *= left2right_damp;
////            }
//
//            for (int i = 1; i < abs(OneStepGoBack) / turn_step; i++)
//            {
//                ctl_msg.data = std::vector<double>{searching_direction * turn_step, turn_speed};  //向原先转过的相反的方向转
//                turning_pub.publish(ctl_msg);
//            }
//            方法二通过一帧一步转过去
            n_turn_step--;
            int turning_direction = theta < 0 ? 1 : -1;
            ctl_msg.data = std::vector<double>{turning_direction * turn_step , turn_speed};  //向原先转过的相反的方向转
            turning_pub.publish(ctl_msg);

            if (n_turn_step > 0)
                return;

            moving_mode = tracking;
            delay_frame = 2;  //跳过帧，等待模式成功转换
            avoid_loop = false;
        }
    }
}

SysCtrl::~SysCtrl(){
    delete searcher;
}

//todo --- 这些是淘汰掉没用上的函数------------------------------------------------
void SysCtrl::GetAndPubImg_jetcam(const sensor_msgs::CompressedImageConstPtr &msg)
{
    try {
        cv_bridge::CvImagePtr cv_ptr_compressed = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        current_left_img = cv_ptr_compressed->image;

        cv::imshow("imgCallback", current_left_img);
        if(cv::waitKey(1)=='q')
            ros::shutdown();
        cv::resize(current_left_img, current_left_img, Size(224*2, 224));
        sensor_msgs::ImagePtr sub_msg;

        if(msg->header.stamp.sec%2==0)
            sub_msg = cv_bridge::CvImage(msg->header, "bgr8", current_left_img.colRange(0, 224)).toImageMsg();
        else
            sub_msg = cv_bridge::CvImage(msg->header, "bgr8", current_left_img.colRange(224, 224*2)).toImageMsg();
        sub_img_pub.publish(sub_msg);
    }
    catch (cv_bridge::Exception &e) {
        //ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void SysCtrl::cam_turn(){
    std_msgs::Float64MultiArray ctl_msg;

    switch (cam_mode)
    {
        case search_bottle:{
            break;
        }
        case look_obstacle:{
            double body_step_turn90 = 15; //body转90度所需的步数
            double scale_cam_body = (1300.0/body_step_turn90); //换算成相机所需单位步数
            //相机转角 = 机体转角换算成的相机单位转角 *  机体所转次数 * 当前迭代增加
            ctl_msg.data =
                std::vector<double>{-(theta / turn_step) * scale_cam_body * cam_avoid_iter, init_position[1] + 150};

            //直走的时候转角加大回头看
            cam_avoid_iter += 0.05;
            cam_avoid_iter = cam_avoid_iter >= 1.4 ? 1.4 : cam_avoid_iter;

            cam_turn_pub.publish(ctl_msg);
            cout << "look_obstacle ========================" << -(theta / turn_step) * scale_cam_body << endl;
            cout << "body   turned ========================" <<  (theta / turn_step) << endl;
            break;
        }
        case  look_road:{
            ctl_msg.data = init_position;
            cam_turn_pub.publish(ctl_msg);
            break;
        }
    }
}

void SysCtrl::GetDepthCompressed_mynteye(const sensor_msgs::CompressedImageConstPtr &depth_msg)
{

    cv_bridge::CvImagePtr cv_ptr_img_left = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::MONO16);
    current_depth = cv_ptr_img_left->image;

    cv::imshow("mynteye depth", current_depth);

//    int key = cv::waitKey(1);
//    if (key == 'q')
//        ros::shutdown();
//    else if(key == 's'){
//        imwrite("/home/lab/Project/RobotPrj/output/depth/depth.jpg",current_depth);
//    }

}

void SysCtrl::GetDepthRaw_mynteye(const sensor_msgs::ImageConstPtr &depth_msg)
{
    current_depth_raw = cv_bridge::toCvShare(depth_msg, "mono16")->image;

    double min_est;
    double max_est;
    int minIdx;
    int maxIdx;

    cv::minMaxIdx(current_depth_raw, &min_est, &max_est, &minIdx, &maxIdx);
//    cv::applyColorMap((65535/(max_est))*current_depth_raw, current_depth_raw, cv::COLORMAP_OCEAN);//转换成颜色编码的深度图
    cv::imshow("raw depth", current_depth_raw);

//    int key = cv::waitKey(1);
//    if (key == 'q')
//        ros::shutdown();
//    else if(key == 's'){
//        imwrite("/home/lab/Project/RobotPrj/output/depth/depth.jpg",current_depth_raw);
//    }

    cvWaitKey(1);
}

void SysCtrl::ImageCallbackRaw(const sensor_msgs::ImageConstPtr& msg)
{
    try {
// image = cv_bridge::toCvShare(msg, "bgr8")->image;
// cv::imshow("view", image);
//  cvWaitKey(1);
        cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
        cvWaitKey(1);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void SysCtrl::avoid_obstacle_callback(const std_msgs::Float32MultiArrayConstPtr &msg)
{
    std_msgs::Float64MultiArray ctl_msg;

    curr_prob_r = msg->data[0];
    curr_prob_l = msg->data[1];

    prob_l.push_back(curr_prob_l);
    prob_r.push_back(curr_prob_r);

    tol_prob_r = tol_prob_r + prob_r.back() - prob_r.front();
    tol_prob_l = tol_prob_l + prob_l.back() - prob_l.front();

    prob_r.pop_front();
    prob_l.pop_front();

//    int x = (tol_prob_r + tol_prob_l) * 10 / win_size; //用于显示


//    for (int i = 0; i < x; i++)
//    {
//        if (i < 10)
//            cout << "-";
//        else if (i < 15)
//            cout << "+";
//        else
//            cout << "*";
//    }
//    cout << endl;


//	单个模态的避障运动
//	cout<<curr_prob_l<<"---"<<curr_prob_r<<endl;
//	cout<<tol_prob_r/win_size<<"---"<<tol_prob_l/win_size<<endl;
//
//	std_msgs::Float64MultiArray ctl_msg;
//	if (tol_prob_r > 0.78 * win_size || tol_prob_l > 0.78 * win_size) {
//			ctl_msg.data = std::vector<double>{0.005, 0.8};
//			turnleft_pub.publish(ctl_msg);
//	}
//	else {
//		ctl_msg.data = std::vector<double>{0.005, 0.8};
//		goforward_pub.publish(ctl_msg);
//	}

//  两个模态的避障
//    if(tol_prob_r > thres_val || tol_prob_l > thres_val)
//        direction = tol_prob_r > thres_val ? 1 : -1;
//    else direction = 0;
//
//    if (direction != 0)
//    {
//        ctl_msg.data = std::vector<double>{direction * turn_step, turn_speed};
//        turning_pub.publish(ctl_msg);
//    }
//    else
//    {
//        ctl_msg.data = std::vector<double>{stright_step, stright_speed};
//        goforward_pub.publish(ctl_msg);
//    }


//多个模态避障:test1

    if(tol_prob_r > thres_val || tol_prob_l > thres_val || curr_distance < 20.0)
//    if(  curr_distance < 20.0  )
        obs_direction = tol_prob_r > thres_val ? 1 : -1;

    else obs_direction = 0;

    if(delay_frame!=0){
        ctl_msg.data = std::vector<double>{stright_step*(0.45), stright_speed};
        goforward_pub.publish(ctl_msg);
        delay_frame--;
        return;
    }


//    if (!avoid_loop){  //如果当前状态不是摆脱障碍物的状态
//
//        if (direction != 0 )  //如果当前遇到障碍物
//        {
//            if(moving_mode==tracking){  //恰好之前一次是没有障碍
//                moving_mode = turn;  //将当前状态改为旋转模式
//                theta = 0;  //重设当前旋转角
//            }
//
//            if(moving_mode==turn){
//                theta += direction*turn_step;  //旋转角按步长叠加
//                ctl_msg.data = std::vector<double>{direction*turn_step, turn_speed};
//                cout<<"direction*turn_step = "<<direction*turn_step<<endl;
//                turning_pub.publish(ctl_msg);
//            }
//
//        }else{  //当前显示没有障碍物
//
//            if(moving_mode==turn){  //如果之前一直是旋转状态，步长在叠加状态
//                avoid_loop = true;  //开启相机搜寻模式，直到摆脱障碍物
//                body_avoid_iter = 20;
////                delay_frame = 3;  //跳过接下来的帧，为相机转过方向延时
//                cam_mode = look_obstacle;
//                delay_frame = 15;
//                cam_turn();  //将相机转向障碍物方向
//            }
//
//            if(moving_mode==tracking){  //如果当前状态是直走，无障碍物，那么继续直走
//                ctl_msg.data = std::vector<double>{stright_step, stright_speed};
//                goforward_pub.publish(ctl_msg);
//                cam_mode = look_road;
//                cam_turn();
//            }
//        }
//
//    }else{ //摆脱障碍物的状态，继续往前走，直到两侧都没有障碍物
//
//        if(direction!=0 || body_avoid_iter>0 ){ //避障方向还能搜索到障碍物，继续直走
//            ctl_msg.data = std::vector<double>{stright_step, stright_speed};
//            goforward_pub.publish(ctl_msg);
//            cam_turn();
//            body_avoid_iter--;
//        }else{  //障碍摆脱，重设当前状态，并且将机器人转回原先方向
//            cam_avoid_iter = 1;  //重设为1
//            //一步走theta角是回不去的，单步回去必须有一个倍数相乘
//            double scale_multi2one = 1.0;
//            double OneStepGoBack = -theta*scale_multi2one;
//
//            //左右轮是有阻尼的，需要进行差速调节
//            double left2right_damp = (51.0/56.0);
//
//            if(theta<0){
//                OneStepGoBack /= left2right_damp;
//            }
//
//            for(int i = 1 ;i < OneStepGoBack/stright_step; i++){
//                ctl_msg.data = std::vector<double>{stright_step, turn_speed };  //向原先转过的相反的方向转
//                turning_pub.publish(ctl_msg);
//            }
//
//            moving_mode = tracking;
//            cam_mode = look_road;  //相机转为看路模式
//            cam_turn();
////            delay_frame = 3;  //跳过10帧，等待模式成功转换
//            avoid_loop = false;
//        }
//    }

//  多个模态的避障 test2
    if (!avoid_loop){  //如果当前状态不是摆脱障碍物的状态

        if (obs_direction != 0 )  //如果当前遇到障碍物
        {
            if(moving_mode==tracking){  //恰好之前一次是没有障碍
                moving_mode = turn;  //将当前状态改为旋转模式
                theta = 0;  //重设当前旋转角
            }

            if(moving_mode==turn){
                theta += obs_direction*turn_step;  //旋转角按步长叠加
                ctl_msg.data = std::vector<double>{obs_direction*turn_step, turn_speed};
                cout << "direction*turn_step = " << obs_direction*turn_step << endl;
                turning_pub.publish(ctl_msg);
            }

        }else{  //当前显示没有障碍物

            if(moving_mode==turn){  //如果之前一直是旋转状态，步长在叠加状态
                avoid_loop = true;  //开启相机搜寻模式，直到摆脱障碍物
                body_avoid_iter = 20;
//                delay_frame = 3;  //跳过接下来的帧，为相机转过方向延时
//                cam_mode = look_obstacle;
//                delay_frame = 15;
//                cam_turn();  //将相机转向障碍物方向
            }

            if(moving_mode==tracking){  //如果当前状态是直走，无障碍物，那么继续直走
                ctl_msg.data = std::vector<double>{stright_step, stright_speed};
                goforward_pub.publish(ctl_msg);
            }
        }
    }else{ //摆脱障碍物的状态，继续往前走，直到两侧都没有障碍物

        if ( body_avoid_iter > 0 )
        {
            //先走 body_avoid_iter 步
            ctl_msg.data = std::vector<double>{stright_step, stright_speed};
            goforward_pub.publish(ctl_msg);
            body_avoid_iter--;
        }
        else
        {  //障碍摆脱，重设当前状态，并且将机器人转回原先方向
            cam_avoid_iter = 1;  //重设为1
            //一步走theta角是回不去的，单步回去必须有一个倍数相乘
            double scale_multi2one = 1.8;
            double OneStepGoBack = -theta * scale_multi2one;

            //左右轮是有阻尼的，需要进行差速调节
            double left2right_damp = (51.0 / 56.0);

            if (theta < 0)
            {
                OneStepGoBack /= left2right_damp;
            }

            for (int i = 1; i < OneStepGoBack / stright_step; i++)
            {
                ctl_msg.data = std::vector<double>{stright_step, turn_speed};  //向原先转过的相反的方向转
                turning_pub.publish(ctl_msg);
            }
            moving_mode = tracking;
            delay_frame = 3;  //跳过10帧，等待模式成功转换
            avoid_loop = false;
        }
    }
}

void SysCtrl::GetIMU_callback(const sensor_msgs::ImuConstPtr &imu_msg){
    double t = imu_msg->header.stamp.toSec();
    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Vector3d acc(dx, dy, dz);
    Vector3d gyr(rx, ry, rz);
    double dt = t - latest_time;
    fastPredictIMU(t, acc, gyr);
}

void SysCtrl::fastPredictIMU(double t, Eigen::Vector3d linear_acceleration, Eigen::Vector3d angular_velocity){
    double dt = t - latest_time;
    latest_time = t;
    Eigen::Vector3d un_acc_0 = latest_Q * (latest_acc_0 - latest_Ba) - g;
    Eigen::Vector3d un_gyr = 0.5 * (latest_gyr_0 + angular_velocity) - latest_Bg;
    latest_Q = latest_Q * Utility::deltaQ(un_gyr * dt);
    Matrix3d curr_R = latest_Q.toRotationMatrix();
    Vector3d ref_x;

    ref_x << 0, 1, 0;
//    ref_x << 0, 0, 1;
    double angle = 10*ref_x.dot(curr_R.col(0)) / curr_R.col(0).norm();
    m_buf.lock();
    if(sin_x_imu.size()<win_size_imu){
        sin_x_imu.push(angle);
    }
    else{
        sin_x_imu.pop();
        sin_x_imu.push(angle);
    }
    m_buf.unlock();

    cout<<"------------angle:"<<angle<<endl;

}
