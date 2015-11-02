#include <ros/ros.h>

#include "dji_sdk/rel_pos.h"
#include "dji_sdk/euler.h"
#include "dji_sdk/set_target_point.h"

#include "dji_ros/attitude.h"

#include <cmath>
#include <stdio.h>

ros::Subscriber rel_pos_sub,euler_sub;
ros::ServiceServer set_target_ser;
ros::ServiceClient drone_attitude_client;

typedef float fp32;
typedef double fp64;

fp64 target_yaw,target_x,target_y,target_height;
fp64 	GPS_YAW_P,GPS_YAW_I,GPS_YAW_D,
		GPS_Y_P,GPS_Y_I,GPS_Y_D,
		GPS_X_P,GPS_X_I,GPS_X_D,
		GPS_Z_P,GPS_Z_I,GPS_Z_D;

//This funcation calculate yaw of target_point in degree
fp32 calc_rel_yaw(fp32 tar_x, fp32 tar_y, fp32 plane_x, fp32 plane_y)
{

	fp32 diff_x  = tar_x - plane_x;
	fp32 diff_y  = tar_y - plane_y;
	fp32 tar_yaw = 0.0;

	if((fabs(diff_x) < 0.0000001) && (fabs(diff_y) < 0.0000001))
	{
		tar_yaw = 0.0;			/* UAV has arrived */
	}else{
		tar_yaw = atan2(diff_y,diff_x);
	}
	return tar_yaw;
}

typedef struct
{
	float x;
	float y;
	float z;

	float roll;
	float pitch;
	float yaw;
}uav_t;

static uav_t current_uav,target_uav;
fp32 out_yaw, out_x, out_y, out_z;
void gps_xy_yaw_pid(uav_t current_uav,uav_t target_uav)
{

	/* yaw PID */
	static fp32 sigma_yaw=0;
	static fp32 Pterm,Iterm,Dterm;
	static fp32 err_yaw1=0,err_yaw2=0;
	
	//fp32 err_yaw=(target_uav.yaw-current_uav.yaw) * 180.0 * M_1_PI;
	fp32 err_yaw = target_uav.yaw * 180.0 * M_1_PI;

	Pterm=GPS_YAW_P*err_yaw;

	sigma_yaw+=err_yaw;
	Iterm=GPS_YAW_I*sigma_yaw;

	Dterm=GPS_YAW_D*(err_yaw+err_yaw2-2*err_yaw1);
	err_yaw2=err_yaw1;
	err_yaw1=err_yaw;

	out_yaw=Pterm+Iterm+Dterm;
//	if(out_yaw>100)out_yaw=100;
//	if(out_yaw<-100)out_yaw=-100;
	



	/* x PID */
	static fp32 sigma_x=0;
	static fp32 err_x1=0,err_x2=0;
    fp32 xPterm,xIterm,xDterm;
	fp32 err_x=target_uav.x-current_uav.x;							//north
	

	xPterm=GPS_X_P*err_x;
	sigma_x+=err_x;
	xIterm=GPS_X_I*sigma_x;
	xDterm=GPS_X_D*(err_x+err_x2-2*err_x1);
	err_x2=err_x1;
	err_x1=err_x;
	out_x=xPterm+xIterm+xDterm;

//	if(out_x>15)out_x=15;
//	if(out_x<-15)out_x=-15;

	/* y PID */
	static fp32 sigma_y=0;
	static fp32 err_y1=0,err_y2=0;
    fp32 yPterm,yIterm,yDterm;
	fp32 err_y=target_uav.y-current_uav.y;							//east

	yPterm=GPS_Y_P*err_y;
	sigma_y+=err_y;
	yIterm=GPS_Y_I*sigma_y;
	yDterm=GPS_Y_D*(err_y+err_y2-2*err_y1);
	err_y2=err_y1;
	err_y1=err_y;
	out_y=yPterm+yIterm+yDterm;

	//if(out_y>15)out_y=15;
	//if(out_y<-15)out_y=-15;

	/* z PID */
	static fp32 sigma_z=0;
	static fp32 err_z1=0,err_z2=0;
	fp32 err_z=target_uav.z;//-current_uav.z;							//height
	fp32 zPterm,zIterm,zDterm;

	zPterm=GPS_Z_P*err_z;
	sigma_z+=err_z;
	zIterm=GPS_Z_I*sigma_z;
	zDterm=GPS_Z_D*(err_z+err_z2-2*err_z1);
	err_z2=err_z1;
	err_z1=err_z;
	out_z=zPterm+zIterm+zDterm;

	//if(out_z>3)out_z=3;
	//if(out_z<-2)out_z=-2;


	/*if(fabs(err)>5)
	{
		out_x=0;
		out_y=0;
	}*/



}
bool set_targetCallback(dji_sdk::set_target_point::Request  &req,
                      dji_sdk::set_target_point::Response &res)
{
	target_uav.x=req.x;
	target_uav.y=req.y;
	target_uav.z=req.z;
	target_uav.yaw = req.yaw;
	return true;
}
void rel_posCallback(const dji_sdk::rel_pos::ConstPtr& msg)
{
	current_uav.x=msg->x;
	current_uav.y=msg->y;
	current_uav.z=msg->z;
	gps_xy_yaw_pid(current_uav,target_uav);
}
void eulerCallback(const dji_sdk::euler::ConstPtr& msg)
{
	current_uav.roll=msg->roll;
	current_uav.pitch=msg->pitch;
	current_uav.yaw=msg->yaw;
}

void cb_timer(const ros::TimerEvent &)
{
		dji_ros::attitude  srv_attitude;

		srv_attitude.request.flag = 0x90;
		srv_attitude.request.x= out_x;
		srv_attitude.request.y = out_y;
		srv_attitude.request.z = out_z;
		srv_attitude.request.yaw = out_yaw;
	
		drone_attitude_client.call(srv_attitude);
}

int main(int argc, char **argv)
{

  	ros::init(argc, argv, "point");
  	ros::NodeHandle nh;
  	ros::NodeHandle nh_private("~");

  	nh_private.getParam("GPS_YAW_P", GPS_YAW_P);
	nh_private.getParam("GPS_YAW_I", GPS_YAW_I);
	nh_private.getParam("GPS_YAW_D", GPS_YAW_D);

	nh_private.getParam("GPS_X_P", GPS_X_P);
	nh_private.getParam("GPS_X_I", GPS_X_I);
	nh_private.getParam("GPS_X_D", GPS_X_D);

	nh_private.getParam("GPS_Y_P", GPS_Y_P);
	nh_private.getParam("GPS_Y_I", GPS_Y_I);
	nh_private.getParam("GPS_Y_D", GPS_Y_D);

	nh_private.getParam("GPS_Z_P", GPS_Z_P);
	nh_private.getParam("GPS_Z_I", GPS_Z_I);
	nh_private.getParam("GPS_Z_D", GPS_Z_D);

	target_uav.x=0;
	target_uav.y=0;
	target_uav.z=3;
	target_uav.yaw = 0;

	rel_pos_sub    = nh.subscribe("rel_pos", 10, rel_posCallback);
	euler_sub      = nh.subscribe("euler", 10, eulerCallback);

	drone_attitude_client = nh.serviceClient<dji_ros::attitude>("DJI_ROS/drone_attitude_control");
	
	set_target_ser = nh.advertiseService("set_target_point", set_targetCallback);

	ros::Timer timer = nh.createTimer(ros::Duration(1/50), cb_timer);


  ros::spin();

  return 0;
}