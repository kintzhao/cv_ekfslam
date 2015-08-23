#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

int main(int argc,char** argv)
{
    ros::init(argc,argv,"odompub");
    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom",50);
    ros::Rate r(30.0);
    ros::Time old_time =ros::Time::now();
    ros::Time current_time;
    double odom_X = 0.0;
    while(n.ok())
    {
        ros::Time current_time;
        current_time = ros::Time::now();
        nav_msgs::Odometry odom;
        double dt = (current_time - old_time).toSec();
        odom.header.stamp = current_time ;
        odom.header.frame_id = "odom";

        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = 0.005;
        odom.twist.twist.linear.y = 0.0000;
        odom.twist.twist.angular.z = 0.0001;

        odom_X +=odom.twist.twist.linear.x*dt;

        odom.pose.pose.position.x = odom_X;
        odom.pose.pose.position.y = 0.001;
        odom.pose.pose.position.z = 0.001;
        //geometry_msgs::Quaternion odom_quat =tf
      //  odom.pose.pose.orientation = {1,0,0,0};

        ROS_INFO("x %.3f ,y %.3f ,z %.3f ",odom.pose.pose.position.x ,
                 odom.pose.pose.position.y,odom.pose.pose.position.z );
        odom_pub.publish(odom);

        r.sleep();
        old_time =current_time;
    }
}
