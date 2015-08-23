#include <ros/ros.h>
#include "./class/detectqrcode.h"
#include "./qrslam.h"
#define SELECT_ROBOT_POS_AS_ORIGIN  0 // 0 表示20 mark 为原点
ofstream fmain("main.txt");
int main(int argc, char** argv)
{
//    vector<float> a;
//    a.push_back(1.1);
//    a.push_back(2.3);
//    a.push_back(3.0);
//     std::cout<< a.size() <<std::endl;
//    for (vector<float>::size_type ix = 0; ix != a.size(); ++ix)
//       std::cout<< a[ix]<<" "<<ix <<std::endl;
//    Point2f a(3.0,4.0);
//    float b = norm(a);
//    cout<<b<<endl;

//    cv::Mat a = Mat::ones(3,3,CV_32FC1) *3;
//    std::cout<<a<<std::endl;

//    cv::Mat tran = Mat::eye(3,5,CV_32FC1);
//     a= tran.t() * a * tran;
//    std::cout<<a<<std::endl;

    ros::init(argc, argv, "qr_slam");
    QrSlam qr_slam(argv[1]);

    ros::spin();

    return 0;
}











