#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <husky_highlevel_controller/HuskyHighlevelController.hpp>
#include <math.h>
#include <ros/init.h>
#include <ros/rate.h>
#include <ros/time.h>
#include <rosconsole/macros_generated.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/Marker.h>
#include <sstream>
#include <vector>



namespace husky_highlevel_controller {


HuskyHighlevelController::HuskyHighlevelController(ros::NodeHandle& nodeHandle) :
  nodeHandle_(nodeHandle)
{
	if (!readParameters()) {
	    ROS_ERROR("Could not read parameters." );
	    ros::requestShutdown();
	  }

	  subscriber_ = nodeHandle_.subscribe(subscriberTopic_, subscriberqueueSize_,
	                                      &HuskyHighlevelController::scan_callback_function, this);


	  publisher_ = nodeHandle_.advertise<geometry_msgs::Twist>(publisherTopic_, publisherqueueSize_);

	  vis_pub = nodeHandle_.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );


	  start_stop_subs = nodeHandle_.subscribe( "/start_stop", 5, &HuskyHighlevelController::start_stop_Callback, this);

	  start_stop_pub = nodeHandle_.advertise<std_msgs::Bool>("/start_stop", 1);

	  stop_obstacle_pub = nodeHandle_.advertise<std_msgs::String>("/stop_obstacle", 10);

}



bool HuskyHighlevelController::readParameters()
{
	  if (!nodeHandle_.getParam("subscriber_topic", subscriberTopic_)) return false;
	  ROS_INFO("Got param: %s", subscriberTopic_.c_str());
	  if (!nodeHandle_.getParam("subscriber_queue_size", subscriberqueueSize_)) return false;
	  ROS_INFO("Got param: %d", subscriberqueueSize_);
	  if (!nodeHandle_.getParam("publisher_topic", publisherTopic_)) return false;
	  ROS_INFO("Got param: %s", publisherTopic_.c_str());
	  if (!nodeHandle_.getParam("publisher_queue_size", publisherqueueSize_)) return false;
	  ROS_INFO("Got param: %d", publisherqueueSize_);
  return true;
}


void HuskyHighlevelController::scan_callback_function(const sensor_msgs::LaserScan::ConstPtr& msg)
{

	int size = msg->ranges.size();
	min_range=999;
	min_range_position=0;
	angle_at_min_range=0;
	int i;
	for (i=0;i<size;i++)
	{
		//ROS_INFO("Range at [%d]: [%f]", i, msg->ranges[i]);
		if ((msg->ranges[i] < min_range)&& (msg->ranges[i] >= msg->range_min) && (msg->ranges[i] <= msg->range_max))
		{
			min_range = msg->ranges[i];
			min_range_position=i;
			angle_at_min_range=((msg->angle_min)+(msg->angle_increment)*i);
		}

	}
//	ROS_INFO("Total measurements: %lu",msg->ranges.size());
//	ROS_INFO("Minimum range seen: %lu with value %f:", min_range_position, min_range);
//	ROS_INFO("Angle min: [%f]", msg->angle_min);
//	ROS_INFO("Angle max: [%f]", msg->angle_max);
//	ROS_INFO("Angle increment: [%f]", msg->angle_increment);
//	ROS_INFO("Angle at minimum range seen: [%f]", angle_at_min_range);
	if(angle_at_min_range <0)
	{
		x_pos=min_range*cos(angle_at_min_range);
		y_pos=min_range*sin(angle_at_min_range);
	}
	else
	{
		x_pos=min_range*cos(angle_at_min_range);
		y_pos=min_range*sin(angle_at_min_range);
	}

//	ROS_INFO("The position of the pillar is x: [%f] y: [%f]", x_pos, y_pos);

	     //Sets the loop to publish at a rate of 10Hz
	     ros::Rate rate(10);

	     if(status_ == false){
	    	 husky_angle_controller(4, angle_at_min_range);
	     	} else{
	     		husky_angle_controller(0, 0);
	     }
  		publisher_.publish(cmd_vel_command);

	    pillar_vis_marker_func();
	    husky_obstactle_distance_controller(x_pos, y_pos);



}

void HuskyHighlevelController::start_stop_Callback( const std_msgs::Bool trigger ){
	if(trigger.data == true){
		status_ = true;
	}else{
		status_ = false;
	}
}

void HuskyHighlevelController::husky_angle_controller(float speed, float angle){
	// let the husky go at a constant speed
	// read the angle towards the pillar, and use PID to turn the husky to the pillar (control the angle to 0)
	cmd_vel_command.linear.x = speed;
	cmd_vel_command.angular.z = 0 - angle;

}

void HuskyHighlevelController::husky_obstactle_distance_controller(float x, float y){
	// let the husky go at a constant speed
	// read the angle towards the pillar, and use PID to turn the husky to the pillar (control the angle to 0)
	if (x<0.6 && y < 0.6)
	{
		std_msgs::String msg;
		std::stringstream ss;
		ss << "STOP";
		msg.data = ss.str();
		stop_obstacle_pub.publish(msg);

		//this option is to stop directly using the service instead of an external node
	//	boolmsg.data = true;
	//	start_stop_pub.publish(boolmsg);

	}

}

void HuskyHighlevelController::pillar_vis_marker_func(){
  	   visualization_msgs::Marker marker;
      marker.header.frame_id = "base_link";
      marker.header.stamp = ros::Time();
      marker.ns = "husky_highlevel_controller";
      marker.id = 0;
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = x_pos;
      marker.pose.position.y = y_pos;
      marker.pose.position.z = 1;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;
      marker.color.a = 1.0; // Don't forget to set the alpha!
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      //only if using a MESH_RESOURCE marker type:
      marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
      vis_pub.publish( marker );
      }
HuskyHighlevelController::~HuskyHighlevelController()
{
}


} /* namespace */

