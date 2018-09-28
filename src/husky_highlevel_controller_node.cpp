#include <husky_highlevel_controller/HuskyHighlevelController.hpp>
#include <ros/init.h>
#include <ros/node_handle.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "husky_highlevel_controller");
  ros::NodeHandle nodeHandle("~");

  husky_highlevel_controller::HuskyHighlevelController huskyHighlevelController(nodeHandle);

  ros::spin();
  return 0;
}




