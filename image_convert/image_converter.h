#ifndef IMAGE_CONVERTER_H
#define IMAGE_CONVERTER_H
#include <stdio.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// 将opencv 图像转化为ros中
class ImageConverter
{
public:
    ros::NodeHandle node;
    image_transport::ImageTransport transport;
    image_transport::Publisher  image_pub;
    image_transport::Subscriber image_sub;

    sensor_msgs::Image ros_img;
    cv::Mat cv_img,cv_carmera;
    cv_bridge::CvImage cvi,cv_bridge_carmera;

public:
    ImageConverter(std::string topic_name);
    ImageConverter(std::string topic_name, cv::Mat mat);

     ~ImageConverter();

    void setCVImage(const cv::Mat& mat);
    void imagecallback(const sensor_msgs::ImageConstPtr& msg);
    void convertOnce();
    void convertOnce(cv::Mat image);
};
#endif // IMAGE_CONVERTER_H

