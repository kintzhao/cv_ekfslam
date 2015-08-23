#include "/home/yhzhao/slam_ws/src/ar_localization/image_convert/image_converter.h"
//#include "image_convert.h"
// 将opencv 图像转化为ros中
static const char WINDOW[] = "Image window";
ImageConverter::ImageConverter(std::string topic_name): transport( node)
{
    image_pub=transport.advertise(topic_name, 1);
}

ImageConverter::ImageConverter(std::string topic_name, cv::Mat mat): transport( node)
{
    image_pub=transport.advertise(topic_name, 1);
    mat.copyTo(cv_img);
}

void ImageConverter::setCVImage(const cv::Mat& mat)
{
    mat.copyTo(cv_img);
}

void ImageConverter::convertOnce(cv::Mat mat)
{
    ros::Time time=ros::Time::now();
    cvi.header.stamp = time;
    cvi.header.frame_id = "image";
    cvi.encoding = "bgr8";
    cvi.image = mat;
    cvi.toImageMsg(ros_img);
    image_pub.publish(cvi.toImageMsg());
}
void ImageConverter::convertOnce()
{
    convertOnce(cv_img);
}

ImageConverter::~ImageConverter()
{
 }


/*//void ImageConverter::imagecallback(const sensor_msgs::ImageConstPtr& msg)
//{
//    cv_bridge::CvImagePtr  cv_ptr;
//    cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);

//  //  cv_carmera.copyTo(cv_ptr->image);
//    cv_ptr->image.copyTo(cv_carmera);
//    cv::imshow("WINDOW",cv_carmera);
//    cv::waitKey(0);
//}

//int main(int argc, char** argv)
//{
//    ros::init(argc, argv, "image_converter");
//    ImageConverter imc("out");
//    ros::spin();
//    return 0;
//}
*/
