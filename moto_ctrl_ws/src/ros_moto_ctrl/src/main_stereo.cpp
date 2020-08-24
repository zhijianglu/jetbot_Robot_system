/******************************/
/*        立体匹配和测距        */
/******************************/

#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

const int imageWidth = 752;                             //摄像头的分辨率
const int imageHeight = 480;
Size imageSize = Size(imageWidth, imageHeight);

Mat ImageL, grayImageL;
Mat ImageR, grayImageR;
Mat rectifyImageL, rectifyImageR;

Rect validROIL;//图像校正之后，会对图像进行裁剪，这里的validROI就是指裁剪之后的区域
Rect validROIR;

Mat mapLx, mapLy, mapRx, mapRy;     //映射表
Mat Rl, Rr, Pl, Pr, Q;              //校正旋转矩阵R，投影矩阵P 重投影矩阵Q
Mat xyz;              //三维坐标

Point origin;         //鼠标按下的起始点
Rect selection;      //定义矩形选框
bool selectObject = false;    //是否选择对象

int blockSize = 0, uniquenessRatio =0, numDisparities=0;
Ptr<StereoBM> bm = StereoBM::create(16, 9);

/*
事先标定好的相机的参数
fx 0 cx
0 fy cy
0 0  1
*/
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



//param 1
//Mat T = (Mat_<double>(3, 1) <<
//    -0.0005,
//      0.1216,
//    -0.0036);//T平移向量
////Mat rec = (Mat_<double>(3, 1) << -0.0163   -0.0068   -0.0018);//rec旋转向量
//
//Mat R = (Mat_<double>(3, 3) <<
//       1.0000 ,   0.0019 ,  -0.0068  ,
//       -0.0017,    0.9999,    0.0163 ,
//       0.0068 ,  -0.0162 ,   0.9998
//       );
//  1.0000 ,   0.0019 ,  -0.0068  , -0.0005
//  -0.0017,    0.9999,    0.0163 ,   0.1216
//  0.0068 ,  -0.0162 ,   0.9998  , -0.0036
//  0      ,   0      ,   0       , 1.0000


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

//
//1.0000   -0.0017    0.0068    0.0008
//0.0019    0.9999   -0.0162   -0.1216
//-0.0068    0.0163    0.9998    0.0016
//0         0         0    1.0000


//body_T_cam0: !!opencv-matrix
// -3.0180966817541233e-03, -9.9999488681070337e-01,-1.0570926491226199e-03, 9.1541069249438157e-04,
// 9.9998468142413222e-01, -3.0131577348100347e-03,-4.6430375340722006e-03, -4.5702523237098287e-02,
// 4.6398286064502610e-03, -1.0710895921435160e-03,9.9998866231452266e-01, 1.7457470631784672e-02,
//  0., 0., 0., 1.0
//
//
//body_T_cam1: !!opencv-matrix
// -1.1919206672936777e-03, -9.9996834924577338e-01,-7.8663734850288602e-03, 1.6206491740527510e-04,
// 9.9993176896068103e-01, -1.2832148812256250e-03,1.1610804568679852e-02, 7.6191396362450703e-02,
// -1.1620531325475341e-02, -7.8519975962609879e-03,9.9990164985635566e-01, 1.4611021944042776e-02,
//  0., 0., 0., 1.

//  1.0000 ,   0.0019 ,  -0.0068  , -0.0005
//  -0.0017,    0.9999,    0.0163 ,   0.1216
//  0.0068 ,  -0.0162 ,   0.9998  , -0.0036
//  0      ,   0      ,   0       , 1.0000


/*****立体匹配*****/
void stereo_match(int,void*)
{
    bm->setBlockSize(2*blockSize+5);     //SAD窗口大小，5~21之间为宜
    bm->setROI1(validROIL);
    bm->setROI2(validROIR);
    bm->setPreFilterCap(31);
    bm->setMinDisparity(0);  //最小视差，默认值为0, 可以是负值，int型
    bm->setNumDisparities(numDisparities*16+16);//视差窗口，即最大视差值与最小视差值之差,窗口大小必须是16的整数倍，int型
    bm->setTextureThreshold(10);
    bm->setUniquenessRatio(uniquenessRatio);//uniquenessRatio主要可以防止误匹配
    bm->setSpeckleWindowSize(100);
    bm->setSpeckleRange(32);
    bm->setDisp12MaxDiff(-1);
    Mat disp, disp8;
    bm->compute(ImageL, ImageR, disp);//输入图像必须为灰度图

    disp.convertTo(disp8, CV_8U, 255 / ((numDisparities * 16 + 16)*16.));//计算出的视差是CV_16S格式
    reprojectImageTo3D(disp, xyz, Q, true); //在实际求距离时，ReprojectTo3D出来的X / W, Y / W, Z / W都要乘以16(也就是W除以16)，才能得到正确的三维坐标信息。
    xyz = xyz * 16;
//    cv::flip(disp8,disp8,-1);
//    cv::flip(disp8,disp8,1);
    imshow("disparity", disp8);
}

/*****描述：鼠标操作回调*****/
static void onMouse(int event, int x, int y, int, void*)
{
    if (selectObject)
    {
        selection.x = MIN(x, origin.x);
        selection.y = MIN(y, origin.y);
        selection.width = std::abs(x - origin.x);
        selection.height = std::abs(y - origin.y);
    }

    switch (event)
    {
        case EVENT_LBUTTONDOWN:   //鼠标左按钮按下的事件
            origin = Point(x, y);
            selection = Rect(x, y, 0, 0);
            selectObject = true;
            cout << origin <<"in world coordinate is: " << xyz.at<Vec3f>(origin) << endl;
            break;
        case EVENT_LBUTTONUP:    //鼠标左按钮释放的事件
            selectObject = false;
            if (selection.width > 0 && selection.height > 0)
                break;
    }
}
string path_left = "/home/lab/Project/RobotPrj/output/imgs/stereo_left.png";
string path_right = "/home/lab/Project/RobotPrj/output/imgs/stereo_right.png";

/*****主函数*****/
int main()
{
    /*
    立体校正
    */
//    Rodrigues(rec, R); //Rodrigues变换

    stereoRectify(cameraMatrixL, distCoeffL, cameraMatrixR, distCoeffR, imageSize, R, T, Rl, Rr, Pl, Pr, Q, CALIB_ZERO_DISPARITY,
                  0, imageSize, &validROIL, &validROIR);


    /*
    读取图片
    */
    ImageL = imread(path_left, CV_8UC1);
    ImageR = imread(path_right, CV_8UC1);

    Mat imgs;
    cv::hconcat(ImageL,ImageR,imgs);
    imshow("raw img", imgs);


    namedWindow("disparity", CV_WINDOW_AUTOSIZE);
    // 创建SAD窗口 Trackbar
    createTrackbar("BlockSize:\n", "disparity",&blockSize, 8, stereo_match);
    // 创建视差唯一性百分比窗口 Trackbar
    createTrackbar("UniquenessRatio:\n", "disparity", &uniquenessRatio, 50, stereo_match);
    // 创建视差窗口 Trackbar
    createTrackbar("NumDisparities:\n", "disparity", &numDisparities, 16, stereo_match);
    //鼠标响应函数setMouseCallback(窗口名称, 鼠标回调函数, 传给回调函数的参数，一般取0)
    setMouseCallback("disparity", onMouse, 0);
    stereo_match(0,0);

    waitKey(0);
    return 0;

}
