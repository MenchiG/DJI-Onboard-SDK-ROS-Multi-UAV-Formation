#include <ros/ros.h>

#include "dji_sdk/rel_pos.h"
#include "dji_sdk/euler.h"
#include "dji_sdk/set_target_point.h"

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>


#include <tf/transform_datatypes.h>

#include <stdio.h>
#include <cmath>

using namespace visualization_msgs;
using namespace interactive_markers;

typedef float fp32;
typedef double fp64;

typedef struct
{
	float x;
	float y;
	float z;

	float roll;
	float pitch;
	float yaw;
}uav_pos_state;

static std::string uav_id;

boost::shared_ptr<InteractiveMarkerServer> server;
MenuHandler menu_handler;

ros::Subscriber rel_pos_sub,euler_sub,uav1_rel_pos_sub,uav1_euler_sub;
ros::ServiceClient follow_client;

InteractiveMarker int_marker;
InteractiveMarkerControl box_control;
InteractiveMarkerControl rotate_control;


tf::StampedTransform lock_transform;

double lock_linear;
double lock_angular;

bool lock_flag = false;

static uav_pos_state uav1_pos, current_pos;
static uint8_t rel_pos_flag = 0;
static uint8_t euler_flag = 0;

#define DEBUG

#ifdef DEBUG
ros::Publisher debug_rel_pos_pub;
#endif

void processFeedback(
    const InteractiveMarkerFeedbackConstPtr &feedback )
{
    if(lock_flag == false)
    {
        dji_sdk::set_target_point set_target_cmd;
        /* to convey the cooridnate from North-East-Sky (not obey the right hand rule)
        to East-North-Sky in rviz we exchange x and y, roll and pitch, and positive direction of Yaw*/
        set_target_cmd.request.x = feedback->pose.position.y;
        set_target_cmd.request.y = feedback->pose.position.x;
        set_target_cmd.request.z = feedback->pose.position.z;

        set_target_cmd.request.yaw = -tf::getYaw(feedback->pose.orientation);

        follow_client.call(set_target_cmd);

        #ifdef DEBUG
        if(uav_id != "uav1")
        {
              dji_sdk::rel_pos debug_pos_msg;

              debug_pos_msg.x = feedback->pose.position.y;
              debug_pos_msg.y = feedback->pose.position.x;
              debug_pos_msg.z = feedback->pose.position.z;

              debug_rel_pos_pub.publish(debug_pos_msg);
        }
        #endif
    }
}

void lockFeedback(
    const InteractiveMarkerFeedbackConstPtr &feedback )
{
    double diffx,diffy;
    MenuHandler::EntryHandle handle = feedback->menu_entry_id;
    MenuHandler::CheckState state;
    menu_handler.getCheckState(handle, state);
    /* this state is which before the click */
    if(state == MenuHandler::UNCHECKED)                                             //lock
    {
        menu_handler.setCheckState(handle, MenuHandler::CHECKED);
        lock_flag = true;
        int_marker.header.frame_id = uav_id;
        int_marker.header.stamp= ros::Time(0);

        int_marker.pose.position.x = 0;
        int_marker.pose.position.y = 0;
        int_marker.pose.position.z = 0;

        int_marker.controls.clear();
        int_marker.controls.push_back( box_control );

        diffx = current_pos.x - uav1_pos.x;
        diffy = current_pos.y - uav1_pos.y;
        lock_angular = atan2(diffy, diffx);
        lock_linear =  sqrt(pow(diffy,2)+pow(diffx,2));


    }else{
        menu_handler.setCheckState(handle, MenuHandler::UNCHECKED);
        lock_flag = false;
        int_marker.header.frame_id = "/world";
        int_marker.header.stamp= ros::Time::now();

        int_marker.pose.position.y = current_pos.x;
        int_marker.pose.position.x = current_pos.y;
        int_marker.pose.position.z = current_pos.z;

        rotate_control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(rotate_control);

        rotate_control.interaction_mode = InteractiveMarkerControl::MOVE_ROTATE;
        int_marker.controls.push_back(rotate_control);

    }
    server->insert(int_marker);
    menu_handler.reApply(*server);
    ros::Duration(1.0).sleep();
    server->applyChanges();
}

void uav1_eulerCallback(const dji_sdk::euler::ConstPtr& msg)
{

    uav1_pos.roll=msg->roll;
    uav1_pos.pitch=msg->pitch;
    uav1_pos.yaw=msg->yaw;

}

void uav1_rel_posCallback(const dji_sdk::rel_pos::ConstPtr& msg)
{

	uav1_pos.x=msg->x;
	uav1_pos.y=msg->y;
	uav1_pos.z=msg->z;

}

void eulerCallback(const dji_sdk::euler::ConstPtr& msg)
{

    current_pos.roll=msg->roll;
    current_pos.pitch=msg->pitch;
    current_pos.yaw=msg->yaw;

    euler_flag = 1;
}

void rel_posCallback(const dji_sdk::rel_pos::ConstPtr& msg)
{

    current_pos.x=msg->x;
    current_pos.y=msg->y;
    current_pos.z=msg->z;

    rel_pos_flag = 1;
}



int main(int argc, char **argv)
{
  	ros::init(argc, argv, "follow");
  	ros::NodeHandle nh;
    uav_id = ros::this_node::getName().substr(1,4);             /* extract uav_id(namespace) of this node need to be modfied*/

	rel_pos_sub    = nh.subscribe("rel_pos", 10, rel_posCallback);
	euler_sub		=nh.subscribe("euler", 10, eulerCallback);

    if (uav_id != "uav1")
    {
        uav1_rel_pos_sub    = nh.subscribe("/uav1/rel_pos", 10, uav1_rel_posCallback);
        uav1_euler_sub       =nh.subscribe("/uav1/euler", 10, uav1_eulerCallback);
    }

	#ifdef DEBUG
	debug_rel_pos_pub = nh.advertise<dji_sdk::rel_pos>("rel_pos", 10);
	#endif

    server.reset(new InteractiveMarkerServer("/interactive"));

	follow_client =nh.serviceClient<dji_sdk::set_target_point>("set_target_point");

	dji_sdk::set_target_point my_pos_srv;

    menu_handler.setCheckState(menu_handler.insert( "Lock", &lockFeedback ), MenuHandler::UNCHECKED);


    int_marker.header.frame_id = "/world";
    int_marker.header.stamp= ros::Time::now();
    int_marker.name = uav_id;
    int_marker.description = uav_id;

    Marker box_marker;
    box_marker.type = Marker::CUBE;
    box_marker.scale.x = 0.45;
    box_marker.scale.y = 0.45;
    box_marker.scale.z = 0.45;
    box_marker.color.r = 0.5;
    box_marker.color.g = 0.5;
    box_marker.color.b = 0.5;
    box_marker.color.a = 1.0;

    box_control.interaction_mode = InteractiveMarkerControl::MENU;
    box_control.markers.push_back( box_marker );
    box_control.always_visible = true;
    int_marker.controls.push_back( box_control );


    rotate_control.orientation.w = 1;
    rotate_control.orientation.x = 0;
    rotate_control.orientation.y = 1;
    rotate_control.orientation.z = 0;

    rotate_control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(rotate_control);

    rotate_control.interaction_mode = InteractiveMarkerControl::MOVE_ROTATE;
    int_marker.controls.push_back(rotate_control);


    server->insert(int_marker, &processFeedback);
    menu_handler.apply(*server, int_marker.name);
    server->applyChanges();

	while(ros::ok())
	{

		//if(rel_pos_flag == 1 && euler_flag == 1)
		{

            if(lock_flag == true)
            {
                fp32  diff_ang =  lock_angular + uav1_pos.yaw;
                fp32  diff_y = lock_linear * sin(diff_ang);
                fp32  diff_x = lock_linear * cos(diff_ang);

                my_pos_srv.request.x = uav1_pos.x + diff_x;
                my_pos_srv.request.y = uav1_pos.y + diff_y;
                my_pos_srv.request.z = uav1_pos.z;
                my_pos_srv.request.yaw = uav1_pos.yaw;

                follow_client.call(my_pos_srv);
                #ifdef DEBUG

                dji_sdk::rel_pos debug_pos_msg;
                debug_pos_msg.x = uav1_pos.x + diff_x;
                debug_pos_msg.y = uav1_pos.y + diff_y;
                debug_pos_msg.z = uav1_pos.z;

                debug_rel_pos_pub.publish(debug_pos_msg);

                #endif

			}else{					/* unlock */
				/* all has been down in processFeedback() */
            }
            rel_pos_flag = 0;
            euler_flag = 0;
        }
        ros::spinOnce();
    }
	server.reset();
  	return 0;
}