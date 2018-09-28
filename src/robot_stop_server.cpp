/*
 * robot_stop_server.cpp
 *
 *  Created on: Sep 26, 2018
 *      Author: milt
 */

#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/service_server.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>
#include <string>

ros::Publisher start_stop_publ;
std_msgs::Bool boolmsg;

bool stop_(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response){
	if(request.data == false){
		boolmsg.data = false;
		start_stop_publ.publish(boolmsg);

		response.success = false;
		response.message = "Not stopped! ";

	}
	if(request.data == true){
		//start_stop_publ.publish(true);
		boolmsg.data = true;
		start_stop_publ.publish(boolmsg);
		response.success = true;
		response.message = "Stopped! ";

	}
	return true;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "robot_start_stop_server");
	ros::NodeHandle nh;
	start_stop_publ = nh.advertise<std_msgs::Bool>("/start_stop", 1);
	ros::ServiceServer service = nh.advertiseService("robot_start_stop", stop_);
	ros::spin();
	return 0;
}

