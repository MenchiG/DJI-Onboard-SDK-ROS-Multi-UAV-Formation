#include "ros/ros.h"


#include "dji_sdk/euler.h"

#include "dji_ros/attitude_quad.h"

#include <cmath>

#define C_EARTH (double) 6378137.0


typedef float fp32;
typedef double fp64;

ros::Subscriber quat_sub;
ros::Publisher  euler_pub;
void quatCallback(const dji_ros::attitude_quad::ConstPtr& msg)
{
    fp32 x=msg->q0;
    fp32 y=msg->q1;
    fp32 z=msg->q2;
    fp32 w=msg->q3;

    dji_sdk::euler euler_msg;
    euler_msg.roll  = atan2(2.0 * (w * z + x * y) , 1.0 - 2.0 * (y * y + z * z));
    euler_msg.pitch = asin(2.0 * (z * x - w * y));
    euler_msg.yaw   = atan2(2.0 * (w * x + y * z) , - 1.0 + 2.0 * (x * x + y * y));

    euler_msg.roll  = euler_msg.roll;
    euler_msg.pitch = euler_msg.pitch;
    euler_msg.yaw   = euler_msg.yaw;

    euler_pub.publish(euler_msg);
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "quat");

    ros::NodeHandle nh;

    quat_sub = nh.subscribe("DJI_ROS/attitude_quad", 10, quatCallback);
    euler_pub = nh.advertise<dji_sdk::euler>("euler", 10);

    ros::spin();

    return 0;
}