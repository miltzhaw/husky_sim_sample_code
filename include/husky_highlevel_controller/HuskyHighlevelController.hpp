#pragma once

#include <geometry_msgs/Twist.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <string>

#define pi 3.1419265358

namespace husky_highlevel_controller {

/*!
 * Class containing the Husky Highlevel Controller
 */
class HuskyHighlevelController {
public:
	/*!
	 * Constructor.
	 */
	HuskyHighlevelController(ros::NodeHandle& nodeHandle);

	/*!
	 * Destructor.
	 */
	virtual ~HuskyHighlevelController();


private:
	  /*!
	   * Reads and verifies the ROS parameters.
	   * @return true if successful.
	   */
	  bool readParameters();

	  /*!
	   * ROS topic callback method.
	   * @param message the received message.
	   */
	  void scan_callback_function(const sensor_msgs::LaserScan::ConstPtr& msg);

	  void pillar_vis_marker_func();
	  //! ROS node handle.
	  ros::NodeHandle& nodeHandle_;

	  //! ROS topic subscriber.
	  ros::Subscriber subscriber_;

	  //! ROS topic name to subscribe to.
	  std::string subscriberTopic_;

	  //! ROS topic name to subscribe to.
	  int subscriberqueueSize_;

	  //! ROS topic subscriber.
	  ros::Publisher publisher_;

	  //! ROS topic name to subscribe to.
	  std::string publisherTopic_;


	  //! ROS topic name to subscribe to.
	  int publisherqueueSize_;

	  ros::Publisher vis_pub;

		double min_range;
		int min_range_position;
		double angle_at_min_range;
		double x_pos;
		double y_pos;

		void husky_angle_controller(float speed, float angle);
		void husky_obstactle_distance_controller(float speed, float angle);
		geometry_msgs::Twist cmd_vel_command;

	  void start_stop_Callback( const std_msgs::Bool trigger );
	  bool status_;
	  ros::Subscriber start_stop_subs;
	  ros::Publisher start_stop_pub;
	  std_msgs::Bool boolmsg;

	  ros::Publisher stop_obstacle_pub;

};

} /* namespace */
