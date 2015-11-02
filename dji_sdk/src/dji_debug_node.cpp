#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include "dji_sdk/rel_pos.h"
#include "dji_sdk/euler.h"
#include "dji_sdk/set_target_point.h"

#include "geometry_msgs/Quaternion.h"

#include <cmath>



typedef struct
{
    float x;
    float y;
    float z;

    float roll;
    float pitch;
    float yaw;
}uav_t;
uav_t current_pos;

static ros::Publisher marker_pub;
static ros::Subscriber rel_pos_sub;
static ros::ServiceClient follow_client;

visualization_msgs::Marker line_strip;


static bool rel_pos_flag = false;
static bool euler_flag = false;


void rel_posCallback(const dji_sdk::rel_pos::ConstPtr& msg)
{
    current_pos.x=msg->x;
    current_pos.y=msg->y;
    current_pos.z=msg->z;

    rel_pos_flag = true;
}
void eulerCallback(const dji_sdk::euler::ConstPtr& msg)
{
    current_pos.roll=msg->roll;
    current_pos.pitch=msg->pitch;
    current_pos.yaw=msg->yaw;

    euler_flag = true;
}
float x,y,z,w;

int main( int argc, char** argv )
{
    ros::init(argc, argv, "debug");
    ros::NodeHandle n;

    tf::TransformBroadcaster br;
    tf::Transform transform;

    std::string uav_id = ros::this_node::getName().substr(1,4);             /* extract uav_id(namespace) of this node need to be modfied*/

    ros::Subscriber rel_pos_sub,euler_sub;

    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    follow_client =n.serviceClient<dji_sdk::set_target_point>("set_target_point");

    rel_pos_sub = n.subscribe("rel_pos", 10, rel_posCallback);
    euler_sub   = n.subscribe("euler", 10, eulerCallback);

    line_strip.header.frame_id = "/world";
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = "debug";
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;

    line_strip.id = 0;

    line_strip.type = visualization_msgs::Marker::LINE_STRIP;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.1;

    if(uav_id == "uav1")
        line_strip.color.b = 1.0;
    else
        line_strip.color.g = 1.0;
    line_strip.color.a = 1.0;

    while (ros::ok())
    {

        static int count = 0;
        count++;
        if(count > 100)
        {
            count = 0;
            geometry_msgs::Point p;
            if(rel_pos_flag == true)
            {
                /* to convey the cooridnate from North-East-Sky (not obey the right hand rule) to
                East-North-Sky in rviz we exchange x and y, roll and pitch, and positive direction of Yaw*/
                p.x = current_pos.y;
                p.y = current_pos.x;
                p.z = current_pos.z;

                line_strip.points.push_back(p);
                //marker_pub.publish(line_strip);
                rel_pos_flag = false;
            }

            if(euler_flag == true)
            {
                /* to convey the cooridnate from North-East-Sky (not obey the right hand rule) to
                East-North-Sky in rviz we exchange x and y, roll and pitch, and positive direction of Yaw*/
                euler_flag   = false;
            }
            /* update tf */
            transform.setOrigin(tf::Vector3(current_pos.y,current_pos.x,current_pos.z));
            tf::Quaternion q;
            q.setRPY(current_pos.pitch, current_pos.roll, -current_pos.yaw);
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", uav_id));


        }
        ros::spinOnce();
    }
}

