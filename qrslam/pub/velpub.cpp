#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include <geometry_msgs/Twist.h>  // for velocity commands
#include <geometry_msgs/TwistStamped.h>  // for velocity commands

#include <nav_msgs/Odometry.h>
#include "tf/LinearMath/Matrix3x3.h"
#include "geometry_msgs/Quaternion.h"


double x = 0.0;
double y = 0.0;
double theta = 0.0;
double x_start = 0.0;
double y_start = 0.0;
double theta_start = 0.0;

bool flag = false;
int  state=0;
#define CV_PI   3.1415926535897932384626433832795

void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    if(!flag)
    {
        geometry_msgs::Quaternion orientation = msg->pose.pose.orientation;
        tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
        double yaw, pitch, roll;
        mat.getEulerYPR(yaw, pitch, roll);
        theta_start = yaw;

        x_start = msg->pose.pose.position.x;
        y_start = msg->pose.pose.position.y;    // ****坐标系反转  逆转实际为负  测量为正
        flag = true;
        ROS_INFO_STREAM("x-start "<< x_start << " | " << y_start<< " | " << theta_start<< " | "  );
    }
    else
    {
        geometry_msgs::Quaternion orientation = msg->pose.pose.orientation;
        tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
        double yaw, pitch, roll;
        mat.getEulerYPR(yaw, pitch, roll);
        theta = yaw;

        x = msg->pose.pose.position.x;
        y = msg->pose.pose.position.y;    // ****坐标系反转  逆转实际为负  测量为正

        ROS_INFO_STREAM("start "<< x_start << " | " << y_start<< " | " << theta_start<< " end| "<<" "<< x << " | " << y<< " | " << theta<< " | "  );
    }

}

double angleWrap(double angle)
{
    ///这个函数用来实现把角度规划到-pi至pi之间
    if (angle>=CV_PI)
        while (angle >= CV_PI)
        { angle=angle-2*CV_PI;}
    else if (angle<-1.0*CV_PI)
        while (angle < -1.0*CV_PI)
        { angle = angle+2*CV_PI;}
  return angle;
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"odompub");
    ros::NodeHandle n;
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",10);
    ros::Subscriber sub = n.subscribe("/odom", 10, chatterCallback);

    //  ros::Rate r(1);
    ros::Rate loop_rate(5);
    ros::Time current_time;
    geometry_msgs::Twist   vel;

    ros::Time old_time =ros::Time::now();
    while(n.ok())
    {
        //cout<<" "<<x<<""<<x_start<<""<<endl;
        ROS_INFO_STREAM("x x-start "<< x << " | " << x_start);
        switch (state)
        {
        case 0: //前进1米
            if ( abs(x - x_start) < 0.5 )
            {
                vel.angular.z = 0.0;
                vel.linear.x  = 0.1;
                vel.linear.y  = 0;
                vel.linear.z  = 0;
            }
            else
            {
                vel.angular.z = 0.0;
                vel.linear.x  = 0.0;
                vel.linear.y  = 0;
                vel.linear.z  = 0;
                state = 1;
                flag = false;
            }
            break;
        case 1: //原地转90°
            if ( angleWrap(theta - theta_start) < CV_PI/2 )
            {
                vel.angular.z = 0.1;
                vel.linear.x  = 0;
                vel.linear.y  = 0;
                vel.linear.z  = 0;
            }
            else
            {
                vel.angular.z = 0.0;
                vel.linear.x  = 0.0;
                vel.linear.y  = 0;
                vel.linear.z  = 0;
                state = 2;
                flag = false;
            }
            break;

        case 2: //前进0.5米
            if ( abs(y - y_start) < 0.5 )
            {
                vel.angular.z = 0.0;
                vel.linear.x  = 0.1;
                vel.linear.y  = 0;
                vel.linear.z  = 0;
            }
            else
            {
                vel.angular.z = 0.0;
                vel.linear.x  = 0.0;
                vel.linear.y  = 0;
                vel.linear.z  = 0;
                state = 3;
                flag = false;
            }
            break;
        case 3: //原地转90°
            if ( angleWrap(theta - theta_start) < CV_PI/2 )
            {
                vel.angular.z = 0.1;
                vel.linear.x  = 0;
                vel.linear.y  = 0;
                vel.linear.z  = 0;
            }
            else
            {
                vel.angular.z = 0.0;
                vel.linear.x  = 0.0;
                vel.linear.y  = 0;
                vel.linear.z  = 0;
                state = 4;
                flag = false;
            }
            break;
        case 4: //前进1米
            if ( abs(x - x_start) < 0.5 )
            {
                vel.angular.z = 0.0;
                vel.linear.x  = 0.1;
                vel.linear.y  = 0;
                vel.linear.z  = 0;
            }
            else
            {
                vel.angular.z = 0.0;
                vel.linear.x  = 0.0;
                vel.linear.y  = 0;
                vel.linear.z  = 0;
                state = 5;
                flag = false;
            }
            break;
        case 5: //原地转90°
            if ( angleWrap(theta - theta_start) < CV_PI/2 )
            {
                vel.angular.z = 0.1;
                vel.linear.x  = 0;
                vel.linear.y  = 0;
                vel.linear.z  = 0;
            }
            else
            {
                vel.angular.z = 0.0;
                vel.linear.x  = 0.0;
                vel.linear.y  = 0;
                vel.linear.z  = 0;
                state = 6;
                flag = false;
            }
            break;
        case 6: //前进0.5米
            if ( abs(y - y_start) < 0.5 )
            {
                vel.angular.z = 0.0;
                vel.linear.x  = 0.1;
                vel.linear.y  = 0;
                vel.linear.z  = 0;
            }
            else
            {
                vel.angular.z = 0.0;
                vel.linear.x  = 0.0;
                vel.linear.y  = 0;
                vel.linear.z  = 0;
                state = 7;
                flag = false;
            }
            break;
        case 7: //原地转90°
            if ( angleWrap(theta - theta_start) < CV_PI/2 )
            {
                vel.angular.z = 0.1;
                vel.linear.x  = 0;
                vel.linear.y  = 0;
                vel.linear.z  = 0;
            }
            else
            {
                vel.angular.z = 0.0;
                vel.linear.x  = 0.0;
                vel.linear.y  = 0;
                vel.linear.z  = 0;
                state = 8;
                flag = false;
            }
            break;

        default:
            vel.angular.z = 0.0;
            vel.linear.x  = 0.0;
            vel.linear.y  = 0;
            vel.linear.z  = 0;
            break;
        }

        current_time = ros::Time::now();
        vel_pub.publish(vel);
        // r.sleep();
        ros::spinOnce();
        loop_rate.sleep(); ///change
    }
}
