/*
 ============================================================================
 Name        : dji_sdk_node.c
 Author      : Ying Jiahang, Wu Yuwei
 Version     :
 Copyright   : Your copyright notice
 Description :
 ============================================================================
 */
/* ROS */
#include <ros/ros.h>

/* msg */

/* srv */
#include "dji_sdk/cmd_sdk.h"
#include "dji_sdk/set_home.h"

#include "dji_ros/control_manager.h"
#include "dji_ros/action.h"
/* SDK */
#include <stdio.h>

/* MATH for_example */
#include <iostream>

#include <unistd.h>



using namespace ros;



/* ros srv*/
ros::ServiceServer cmd_sdk_server;
ros::ServiceClient set_home_client;
ros::ServiceClient drone_control_manager;
ros::ServiceClient drone_action_client;



bool cmd_sdk_callback(dji_sdk::cmd_sdk::Request &req,
					  dji_sdk::cmd_sdk::Response &res)
{
	std::string uav_id = ros::this_node::getName().substr(1,4);				/* extract uav_id(namespace) of this node need to be modfied*/
	


	if(req.cmd == "set_home"){
		dji_sdk::set_home set_home_cmd;
		if(req.uav == "uav1"){
			set_home_cmd.request.request_home_flag = 0;
		}
		else{
			sleep(2);
			set_home_cmd.request.request_home_flag = 2;
		}
		set_home_cmd.request.longti = 0;
		set_home_cmd.request.lati   = 0;
		set_home_client.call(set_home_cmd);
		return true;
	}
	else if (req.uav == uav_id)
	{
		res.ack = uav_id + " ok\n";
		if 	(req.cmd == "init"){
			dji_ros::control_manager srv_control;
			srv_control.request.control_ability=1;
			drone_control_manager.call(srv_control);
		}else if(req.cmd == "take_off"){
			dji_ros::action srv_action;
			srv_action.request.action=4;
			drone_action_client.call(srv_action);
		}else{
			res.ack = uav_id + " " + req.cmd + " don't know this cmd\n";
		}
	}else{
		res.ack = req.cmd + " no this cmd\n";
	}
}




/*
  * main_function
  */
int main (int argc, char** argv)
{

	printf("Test SDK Protocol demo\n");
	/* initialize ros */
	ros::init(argc, argv, "SDK_node");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");

	cmd_sdk_server  = nh.advertiseService("/cmd", cmd_sdk_callback);

	set_home_client 		= nh.serviceClient<dji_sdk::set_home>("set_home");


	drone_control_manager   = nh.serviceClient<dji_ros::control_manager>("DJI_ROS/obtain_release_control");
	drone_action_client = nh.serviceClient<dji_ros::action>("DJI_ROS/drone_action_control");
	ros::spin();

	return 0;
}
