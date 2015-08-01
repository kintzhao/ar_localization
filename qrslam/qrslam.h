#ifndef QRSLAM_H
#define QRSLAM_H

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/timeb.h>

#include <cstring>
#include <cstdio>
#include <math.h>
#include <iostream>
#include <vector>
#include <map>
#include <string>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "tf/LinearMath/Matrix3x3.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"



#include "simulator.h"  //  g2o相关的库

#include <opencv2/opencv.hpp>
#include <opencv/cv.h>
#include <fstream>

#include "./class/detectqrcode.h"
#include "../image_convert/image_converter.h"

#define undistort 0//1
#define DATA_FUSION_NUM 10          //    滑动均值窗口的大小

#define IS_OPEN_DATA_FILTER 0      //   角速度打开数据滤波
#define IS_OPEN_BUBBLE_FILTER 0      //   角速度打开数据滤波_冒泡：去大小值
#define IS_OPEN_ROBOT_POSE_EKF_FILTER 0      //   robot_pose_ekf
#define SELECT_LANDMARK_NUM 0   //  从2D mark中提取landmark的数量  1只选择中心点 ；0 选择五点
#define IS_OPEN_DYNAMIC_MAP 0   // 0 表示只绘制当前的系统状态landmark,深度复制 ;1表示动态的整个过程
#define DISPLAY_UNDER_MARK_COORDINATE 1   // 0 表示只绘制当前的系统状态landmark,深度复制 ;1表示动态的显示landmark整个过程

using ARToolKitPlus::TrackerSingleMarker;
using ARToolKitPlus::ARMarkerInfo;
using namespace std;

//class Point3ffi
//{
//public:
//    float x;
//    float y;
//    int z;
//    Point3ffi(float x1,float y1,int z1)
//    {
//        x=x1;y=y1;z=z1;
//    }
//};
typedef struct Point3ffi
{
    float x;
    float y;
    int z;
    void init(float x1,float y1,int z1)
    {
        x=x1;
        y=y1;
        z=z1;
    }
}Point3ffi;

typedef struct RobotInfo
{
    double Vx;
    double Vy;
    double V;
    double W;

    double Theta;
    double X;
    double Y;
}RobotInfo;

using namespace std;
using namespace cv;
class QrSlam
{
public :
    QrSlam(char* addr);
    ~QrSlam();

    //**********与机器人位姿相关的变量****************************
    //------------与机器人定位相关的 variables--------------//
    vector<Point3f> landmarks_;            // 2d-->3d
    vector<Point2f> velocities_;          //For velocity model,用来保存直线和角输入,其保存的第一个元素为零，第二个元素代表从第一个位置与第二个位置之间的速度，以此类推
    vector<Point3ffi> observations_;
    vector<Point3f> est_path_points_;       //估计所得路点
    vector< vector<Point2f> > landmarks_system_;       //估计所得路点
    Mat xP; 		                     //当前时刻所估计得到的机器人位姿协方差矩阵

    ///////////////////////EKFSLAM变量
    Mat miu_state ;           //EKF_SLAM中组合状态向量
    Mat miu_convar_p ;           //EKF_SLAM中的组合方差矩阵
    Mat landmark_observed;    //记录所有被观察到的landmark
    vector<int> observed_landmark_num;
    vector<int> Lm_observed_Num;
    int LandmarkSumNum;

    int landmark_have_drawed;

    static const int MAP_BASE_Y_ = 150 ;
    //    static const int MAP_BASE_X_ = 500 ;
    static const int MAP_BASE_X_ = 100 ;

//    const float a1 = 0.087;//0.1;
//    const float a2 = 0.002;//0.1;
//    const float a3 = 0.1203;//0.1;
//    const float a4 = -0.0037;//0.1;

     const float a1 = 0.5;//0.1;
     const float a2 = 0.0032;//0.1;
     const float a3 = 0.0032;//0.1;
     const float a4 = 0.0016;//0.1;


//    const float a5 = 0.1;
//    const float a6 = 0.1;          //a1-a6为速度和里程计模型噪声参数
     const float sigma_r = 0.1;
     const float sigma_phi = 0.1;   //观测模型噪声
     const float p_init_x_ = 0.0001;        // 0.1;//0.1;//1;//100;//0.1;
     const float p_init_y_ = 0.0001;        //0.10;//0.1;//1;//100;//0.1;
     const float p_init_theta_ = 0.02618;    //0.38;//0.1;//0.38;
     const float convar_x_ = 0.0005;        // 0.1;//0.1;//1;//100;//0.1;
     const float convar_y_ = 0.0005;        //0.10;//0.1;//1;//100;//0.1;
     const float convar_theta_ = 0.000685;    //0.38;//0.1;//0.38;

     const float convar_measure[4] = {310.1275, 0, 0, 1.6933 };  //静态下

     const float update_odom_linear = 0.05;
     const float update_odom_angle = 0.05;
     const float stamp_interval = 0.06;
//   const float convar_measure[4] = {10, 0, 0, 1.6933 };  //静态下
//   const float convar_measure[4] = {10, 0, 0, 0.6933 };  //
//   const float convar_measure[4] = {1.9337,0,0,0.0040};  //静态下
    // const float convar_measure[4] = {1.9337,0,0,0.0040};
    //      const float convar_measure[4] ={ 3811.01352137816,126.695330400850,126.695330400850,4.76963060027493};
    //      0.0000    0.0001      0.0001    0.0859    动态：  速度预测下
    //      1.9337    0.0847    0.0847    0.0040      动态下： 线性拟合

    static const int INIT_LOCALIZATION = 20;// 初始定位缓存大小
    const int SELECT_MARK_FOR_INIT_ = 20; // 用作初始定位的landmark id .

    const int CAMERA_FREQUENCE_DIFF_ = 1 ;
    const int ODOMETRY_FREQUENCE_DIFF_ = 1 ;
    RNG   rng;

    float Vd_;                            //控制输入线性速度(m/s),,,这是理想的没有噪声的，真实的我们要加入噪声
    float Wd_;                            //控制输入角速度（rad/s)，，，INSERTPATHPOINT函数里面的Wr和Vr是加入噪声的，真实的速度

    timeb TimeOld_, Time_;
    float delta_t_;                             //控制和观察周期

    void    genObservations();         //定义观测模型
    void    angleWrap(float& angle);
    float   genGaussianValue(float Sigma2);   //generate a Gaussian value with variance sigma2
    void    displayMatrix(Mat matrix) ;
    void    writeMatrix(Mat matrix) ;

    cv::Mat landmark_miu_conver_p_;   //协方差椭圆图时用到

    int mark_num_;
    int visual_mark_num_;

    void ekfSlam_old(float V, float W);
    Point3f PosNow ;
    double  odom_aver(double *Vodom, int n);

public:
    RobotInfo robot_info_;

    vector<QrLandMark> landmark_vector_;
    vector<CPointsFour> landmark5_vector_;
    vector<CPointsFour> mark5_init_vector_;

    int mark5_init_num;


    double Vodom[DATA_FUSION_NUM];
    double Wodom[DATA_FUSION_NUM];
    int    data_filter_num;

    int     odom_i;
    int  odom_init_;
    bool init_EKF_value_;
    bool init_state_;


public:
    ros::NodeHandle   ar_handle;
    ros::Publisher    observation_pub_;
    ros::Subscriber   odometer_sub_;
    ros::Subscriber   odo_combined_sub_;


    image_transport::ImageTransport transport_;
    image_transport::Subscriber qr_image_sub_;

    ros::Time     current_time_, last_time_;
    DetctQrcode*  pQrDetect_;

    //public function
    std::string  int2str(int num);
    std::string  float2str(float num) ;
    void  showImage() ;
    int getNumQrcode(void);
    void  getOdomterCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void  robotPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg );
    void  qrDetectCallback(const sensor_msgs::ImageConstPtr& img_msg);
    int   qr_detect_init_num ;
    int   odom_init_num;
    int   observed_mark_num_old ;

    timeb test_time_old,test_time;

//display
    ImageConverter* raw_img_cvt_ = NULL;
    ImageConverter* robot_img_cvt_ = NULL;
    ImageConverter* slam_img_cvt_ = NULL;


    cv::Mat raw_global_map_ ;
    cv::Mat raw_global_map_flip0_ ;
    cv::Mat cv_camera_;
    cv::Mat qr_detect_;
    void showRobot(Mat &map, RobotInfo robot_info, Scalar rgb);
    void showRobotOrientation(Mat image, RobotInfo robot_info, Scalar rgb, int x_coordinate, int y_coordinate);
    void showRobotOrientation(Mat image, Mat robot_info,Scalar rgb,int x_coordinate,int y_coordinate);
    void showSystemStateLandmark(Mat &map, Scalar rgb);
    void showSystemStateRobot(cv::Mat& map, Scalar rgb);
    void drawCoordinate(cv::Mat& mat);
    void storeData(void );
    bool is_odom_update ;
    bool is_img_update_;
    Point3ffi addObservationPos(ConerPoint landmarktemp , int id);
    //coordinate change
    float coordinate_x_;
    float coordinate_y_;
    float coordinate_angle_;
    bool coordinate_init_finished_;
    bool init_start_localization_;
    void initLocalization( );
    Point3f diffCoordinate(CPointsFour mark_2d,CPointsFourWorld mark_2d_world);

    void  showRobotTriangle(cv::Mat& map, RobotInfo robot_info, Scalar rgb);
    void  qrDetectDataStore( vector<CPointsFour> vector_data );
    bool  vectorIsValid(vector<CPointsFour> vector_data);
    void  dataFilter(double &v, double &w);
    void bubbleSort(double unsorted[]);
    void storeSystemState( Mat systemState);
    void WorldToMark3(Point3f &dst, Point3f src);
    void WorldToMark3(Point3d &dst, Point3d src);
    void WorldToMark2(Point3f &dst, Point3f src);
    void WorldToMark2(Point3d &dst, Point3d src);
    bool stateExtended(int num, int num_old);
    ros::Time odom_stamp;
    ros::Time image_stamp;
    bool IS_MAP_UPDATE;
    void ekfSlam(float V, float W);
    Point2f motionModel(Point2f motion, Mat& system_state, Mat& system_state_convar, int variable_num, float delta_time);
    void updateSystemState(Mat& system_state, Mat& system_state_convar, int variable_num, vector<Point3ffi> observation_Marks_per_image);

};

#endif

