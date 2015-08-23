#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <fstream>
#include <vector>
#include <stdlib.h>
#include <stdio.h>

using namespace std;
ofstream fodom("fodom.txt");

ros::Time current_time,last_time;
double Theta ;

void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
   current_time = ros::Time::now();
   double dt = (current_time - last_time).toSec();
   double X = msg->pose.pose.position.x*100;
   double Y = msg->pose.pose.position.y*100;  // ****坐标系反转  逆转实际为负  测量为正
   double Z = msg->pose.pose.position.z*100;   //  cm


   double Vx = msg->twist.twist.linear.x*100;      //m/s  *s -->m  *100>>>>cm   显示都是以cm为单位的
   double Vy = msg->twist.twist.linear.y*100;  //cm/s

   double W  = msg->twist.twist.angular.z;  //****坐标系反转  逆转实际为负 测量为正  角速度积分计算要注意

   geometry_msgs::Quaternion Q = msg->pose.pose.orientation ;

   Theta += W*dt;
   last_time = current_time ;

   fodom<<"dt x y z vx vy w theta Qxyzw"<<" "<<dt<<" "<<X<<" "<<Y<<" "<<Z<<" "<<Vx<<" "<<Vy<<" "<<W<<" "<<Theta<<" "<<Q.x<<" "<<Q.y<<" "<<Q.z<<" "<<Q.w<<" "<<endl;
   cout<<"dt x y z vx vy w theta Qxyzw"<<" "<<dt<<" "<<X<<" "<<Y<<" "<<Z<<" "<<Vx<<" "<<Vy<<" "<<W<<" "<<Theta<<" "<<Q.x<<" "<<Q.y<<" "<<Q.z<<" "<<Q.w<<" "<<endl;

}



int main(int argc,char **argv)
{
    ros::init(argc, argv, "odomlisten");
    ros::NodeHandle ol;

    Theta = 0.0;
    ros::Subscriber sub = ol.subscribe("odom", 1, chatterCallback);
    ros::spin();
    return 0;
}

