/*
 * robot_start_server.cpp
 *
 *  Created on: Sep 26, 2018
 *      Author: milt
 */

#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <rosconsole/macros_generated.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <string>

ros::ServiceClient client;
void obstacleCallback(const std_msgs::String::ConstPtr& msg)
{
	std_srvs::SetBool service;
	ROS_INFO("I heard: [%s]", msg->data.c_str());
  if(msg->data == "STOP")
  {
	  ROS_INFO("Too close to an obstacle: Send request to stop the robot!");
	  	service.request.data = true;
	  	if (client.call(service)) {
	  		if(service.response.success == true){

	  			ROS_INFO(service.response.message.c_str());
	  		} else{

	  			ROS_INFO(service.response.message.c_str());
	  		}
	  	} else {
	  		ROS_ERROR("Failed to call service emergency_stop");
	  	}
  }
}



int main(int argc, char **argv) {
	ros::init(argc, argv, "robot_start_stop_client");
	ros::NodeHandle nh;
	client = nh.serviceClient<std_srvs::SetBool>("robot_start_stop");


	ros::Subscriber sub = nh.subscribe("/stop_obstacle", 100, obstacleCallback);
	ros::spin();
	return 0;
}

/* The code below would just start and after 100ms stop the robot calling the service

int main(int argc, char **argv) {
	ros::init(argc, argv, "robot_start_stop_client");
	ros::NodeHandle nh;
	ros::ServiceClient client = nh.serviceClient<std_srvs::SetBool>("robot_start_stop");
	std_srvs::SetBool service;


   sleep(100);
	ROS_INFO("Too close to an obstacle: Send request to stop the robot!");
	service.request.data = true;
	if (client.call(service)) {
		if(service.response.success == true){

			ROS_INFO(service.response.message.c_str());
		} else{

			ROS_INFO(service.response.message.c_str());
		}
	} else {
		ROS_ERROR("Failed to call service emergency_stop");
	}
	return 0;
}
*/


