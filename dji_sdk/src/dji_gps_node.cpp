#include <ros/ros.h>

#include "dji_sdk/rel_pos.h"
#include "dji_sdk/set_home.h"

#include "dji_ros/global_position.h"

#define C_EARTH (double) 6378137.0

typedef float fp32;
typedef double fp64;

static fp64 home_lati            = -1.0;
static fp64 home_longti          = -1.0;
static fp64 lati                 = 0.0;
static fp64 longti               = 0.0;
static bool home_set             = false;
static unsigned char health_flag = 0;

ros::Subscriber gps_sub;

ros::ServiceServer set_home_server;
ros::ServiceClient set_home_client;

ros::Publisher rel_pos_pub;
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
bool set_homeCallback(dji_sdk::set_home::Request    &req,
                      dji_sdk::set_home::Response   &res)
{
    std::string uav_id = ros::this_node::getName().substr(1,4);
    if(health_flag>=3)
    {
        if(req.request_home_flag == 0)               /* uav1 will run this part to give other uavs the home pos*/
        {                                            /* a request from other uavs or other uavs*/

            home_set = true;

            home_lati   = lati;
            home_longti = longti;

            res.ack    = "home_set";
            res.lati   = home_lati;
            res.longti = home_longti;

        }else if(req.request_home_flag == 1){        /* uav1 will run this part to give other uavs the home pos*/
            if(home_set == true)                     /* an ack to other uavs or other uavs*/
            {
                res.ack    = "ok";
                res.lati   = home_lati;
                res.longti = home_longti;
            }
            else
            {
                res.ack = "bad";
                res.lati = -1;
                res.longti = -1;
            }                                         
        }
        else{                                      /* other uavs will run this part for requesting home pos from uav1 */
            dji_sdk::set_home req_home_pos;         /* request home pos from UAV1 */

            req_home_pos.request.request_home_flag = 1;

            if(set_home_client.call(req_home_pos)) 
            {
                if(req_home_pos.response.ack == "ok")
                {
                    home_set    = true;
                    home_lati   = req_home_pos.response.lati;
                    home_longti = req_home_pos.response.longti;
                    res.ack     = uav_id + " home set.";
                    res.lati    = home_lati;
                    res.longti  = home_longti;
                }else{
                    home_set = false;
                    res.ack = "uav1 is not ready!!!";
                    res.lati = -1;
                    res.longti = -1;
                }

            }
        }
    }else{

        home_set = false;
        res.ack = "bad_gps";
        res.lati = -1;
        res.longti = -1;
    }
    return true;
}

void gpsCallback(const dji_ros::global_position::ConstPtr& msg)
{

        lati        = msg->latitude;
        longti      = msg->longitude;
        health_flag = msg->health;

        if(home_set == false)
        {
            ROS_WARN_ONCE("There is no home!!!\n");
            ROS_WARN_ONCE("No rel_pos output before setting home!!!\n");
            return;
        }

        //ROS_INFO("health_flag is %d", msg->health_flag);

        fp64 dlati = msg->latitude - home_lati;
        fp64 dlongti= msg->longitude - home_longti;

        dji_sdk::rel_pos rel_pos_msg;

        rel_pos_msg.x = dlati*C_EARTH;
        rel_pos_msg.y = dlongti*C_EARTH*cos(msg->latitude / 2.0+home_lati/2.0);
        rel_pos_msg.z = msg->height;

        rel_pos_pub.publish(rel_pos_msg);

}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "gps");

    ros::NodeHandle nh;

    gps_sub = nh.subscribe("DJI_ROS/global_position", 10, gpsCallback);
    set_home_server = nh.advertiseService("set_home", set_homeCallback);

    set_home_client = nh.serviceClient<dji_sdk::set_home>("/uav1/set_home");

    rel_pos_pub = nh.advertise<dji_sdk::rel_pos>("rel_pos", 10);

    ros::spin();

    return 0;
}