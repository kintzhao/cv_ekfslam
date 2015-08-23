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

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include "simulator.h"  //  g2o相关的库

#include <opencv2/opencv.hpp>
#include <opencv/cv.h>
#include <fstream>

#include "./class/detectqrcode.h"
#include "../image_convert/image_converter.h"

#define undistort 0//1
#define DATA_FUSION_NUM 10          //    滑动均值窗口的大小

#define IS_OPEN_BUBBLE_FILTER 0      //   角速度打开数据滤波_冒泡：去大小值
#define IS_ONE_POINT_AS_LANDMARK 0   //  从2D mark中提取landmark的数量  1只选择中心点 ；0 选择五点
#define IS_OPEN_DYNAMIC_MAP 0   // 0 表示只绘制当前的系统状态landmark,深度复制 ;1表示动态的整个过程
#define DISPLAY_UNDER_MARK_COORDINATE 0   // 0表示在odom坐标系下 ;1表示转换到mark下的显示
#define IS_ALL_SYSTEM_AS_MARK_COORDNATE 0   //PS: 与 DISPLAY_UNDER_MARK_COORDINATE 存在两者分立,不开同时  ekfslam初始就进行转换

typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::Image> slamSyncPolicy;

using ARToolKitPlus::TrackerSingleMarker;
using ARToolKitPlus::ARMarkerInfo;
using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

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

typedef struct OdomMessage
{
    ros::Time  stamp;
    double v;
    double w;
    double theta;
    double x;
    double y;
    void init(double x0,double y0,double theta0,double v0,double w0)
    {
       x = x0;
       y = y0;
       theta = theta0;
       v = v0;
       w = w0;
    }
}OdomMessage;


typedef struct landmarkObservation
{
    int id;
    Point2f measure;
    Point2f predict;
    Point3f robot_odom;
    Point3f robot_predict;
    ros::Time  stamp;
}landmarkObservation;

using namespace std;
using namespace cv;
class QrSlam
{
public :
    QrSlam(char* addr);
    ~QrSlam();

    //**********与机器人位姿相关的变量****************************
    //------------与机器人定位相关的 variables--------------//

    vector<Point2f> velocities_;          //For velocity model,用来保存直线和角输入,其保存的第一个元素为零，第二个元素代表从第一个位置与第二个位置之间的速度，以此类推
    vector<Point3ffi> observations_;

    vector<Point3f> est_path_points_;       //估计所得路点
    vector< vector<Point3f> > landmarks_system_;       //估计所得路点

    ///////////////////////EKFSLAM变量
    Mat miu_state ;           //EKF_SLAM中组合状态向量
    Mat miu_convar_p ;           //EKF_SLAM中的组合方差矩阵

    vector<int> observed_landmark_num;
    vector<int> Lm_observed_Num;

    int landmark_have_drawed;

    static const int MAP_BASE_Y_ = 150 ;
    //    static const int MAP_BASE_X_ = 500 ;
    static const int MAP_BASE_X_ = 100 ;

//    const float a1 = 0.087;//0.1;
//    const float a2 = 0.002;//0.1;
//    const float a3 = 0.1203;//0.1;
//    const float a4 = -0.0037;//0.1;

     const float a1 = 0.001;//0.1;
     const float a2 = 0;//0.1;
     const float a3 = 0;//0.1;
     const float a4 = 0.005;//0.1;


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

//     const float convar_measure[4] = {310.1275, 0, 0, 1.6933 };  //静态下
     const float convar_measure[4] = { 0.1, 0.0, 0.0, 0.0005};  //静态下

//     const float update_odom_linear_ = 4;
     const float update_odom_linear_ = 0.5;
     const float update_odom_angle_  = 0.17;
     const float stamp_interval = 0.005;//0.002;

//   const float convar_measure[4] = {10, 0, 0, 1.6933 };  //静态下
//   const float convar_measure[4] = {10, 0, 0, 0.6933 };  //
//   const float convar_measure[4] = {1.9337,0,0,0.0040};  //静态下
    // const float convar_measure[4] = {1.9337,0,0,0.0040};
    //      const float convar_measure[4] ={ 3811.01352137816,126.695330400850,126.695330400850,4.76963060027493};
    //      0.0000    0.0001      0.0001    0.0859    动态：  速度预测下
    //      1.9337    0.0847    0.0847    0.0040      动态下： 线性拟合

    static const int INIT_LOCALIZATION = 10;// 初始定位缓存大小
    const int SELECT_MARK_FOR_INIT_ = 20; // 用作初始定位的landmark id .

    const int CAMERA_FREQUENCE_DIFF_ = 1 ;
    const int ODOMETRY_FREQUENCE_DIFF_ = 1 ;
    RNG   rng;

    ros::Time time_bag_,time_bag_old;
    float delta_t_;                             //控制和观察周期

    void    getObservations();         //定义观测模型
    void    angleWrap(float& angle);
    float   genGaussianValue(float Sigma2);   //generate a Gaussian value with variance sigma2
    void    displayMatrix(Mat matrix) ;
    void    writeMatrix(Mat matrix) ;
    int visual_mark_num_;
public:
    vector<OdomMessage>  odom_messages_vector;
    vector<CPointsFour> landmark5_pix_vector_;
    vector<CPointsFour> landmark5_meter_vector_;
    vector<CPointsFour> mark5_init_vector_;

    double Vodom[DATA_FUSION_NUM];
    double Wodom[DATA_FUSION_NUM];
    int    data_filter_num;
    bool IS_INIT_FINISH;

public:
    ros::NodeHandle   ar_handle;

    message_filters::Subscriber<nav_msgs::Odometry>* odom_sub_ ;             // topic1 输入
    message_filters::Subscriber<sensor_msgs::Image>* img_sub_;   // topic2 输入
//    TimeSynchronizer<nav_msgs::Odometry, sensor_msgs::Image>* sync_wiki_;       // 同步

    message_filters::Synchronizer<slamSyncPolicy>* sync_;
    image_transport::ImageTransport transport_;
    image_transport::Subscriber qr_image_sub_;

    DetctQrcode*  pQrDetect_;

    //public function
    std::string  int2str(int num);
    std::string  float2str(float num) ;
    void  showImage() ;
    int   getLandmarkNum(void);
    int   observed_mark_num_old ;

//display
    ImageConverter* raw_img_cvt_ = NULL;
    ImageConverter* robot_img_cvt_ = NULL;
    ImageConverter* slam_img_cvt_ = NULL;

    cv::Mat raw_global_map_ ;
    cv::Mat cv_camera_;

    void showRobot(Mat &map, OdomMessage rob_odom, Scalar rgb);
    void showRobotOrientation(Mat image, OdomMessage rob_odom, Scalar rgb, int x_coordinate, int y_coordinate);
    void showRobotOrientation(Mat image, Mat rob_update_mat, Scalar rgb, int x_coordinate, int y_coordinate);
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

    Point3f diffCoordinate(CPointsFour mark_2d,CPointsFourWorld mark_2d_world);
    void  showRobotTriangle(cv::Mat& map, OdomMessage rob_odom, Scalar rgb);

    bool  vectorIsValid(vector<CPointsFour> vector_data);
    void  dataFilter(double &v, double &w);
    void bubbleSort(double unsorted[]);
    void storeSystemState( Mat systemState);
    void WorldToMark3(Point3f &dst, Point3f src);
    void WorldToMark3(Point3d &dst, Point3d src);
    void WorldToMark2(Point3f &dst, Point3f src);
    void WorldToMark2(Point3d &dst, Point3d src);
    void stateExtended(int num, int num_old);
    ros::Time odom_stamp;
    ros::Time image_stamp;
    ros::Time odom_init_stamp; //初始下的时间戳

    Point2f motionModel(Point2f motion, Mat& system_state, Mat& system_state_convar, int variable_num, float delta_time);
    void updateSystemState(Mat& system_state, Mat& system_state_convar, int variable_num, vector<Point3ffi>& observation_Marks_per_image);

    int  Init_mark_in_systemState_j; //用于初始单位的landmark位于系统状态landmark的次序
    cv::Mat global_map_display_; //析构的时候可以保存图片

    int num_EKFSlam_predict_;  //记录预测次数
    int num_EKFSlam_update_;  //记录更新次数
    int num_time_interval;
    vector< vector<landmarkObservation> > landmark_vectors_;
    vector<Point2f> K_vec;
    vector< vector<Point2f> >  Kt_vectors_;
    OdomMessage robot_odom_;

    float odom_theta;
    float odom_theta_old;

    int call_back;
    int ekf_start;
    int ekf_end;

    void combineCallback(const nav_msgs::Odometry::ConstPtr& pOdom, const sensor_msgs::ImageConstPtr& pImg) ;
    void getOdomData(const nav_msgs::Odometry::ConstPtr& odomMsg);
    bool getImgData(const sensor_msgs::ImageConstPtr& imgMsg);
    void pixDataToMetricData();
    bool addInitVectorFull();
    void computerCoordinate();
    void savePixDatas( vector<CPointsFour> vector_data );
    void saveMeterDatas( vector<CPointsFour> vector_data );
    void ekfslam(OdomMessage rob_odom);

};

#endif

