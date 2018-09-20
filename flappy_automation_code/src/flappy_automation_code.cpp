#include "flappy_automation_code/flappy_automation_code.hpp"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "flappy_automation_code");
	ros::NodeHandle node_handle;
	flappy_navigation::FlappyNavigation flappy_navigator(node_handle);

  // Ros spin to prevent program from exiting
  ros::spin();
  return 0;
}
