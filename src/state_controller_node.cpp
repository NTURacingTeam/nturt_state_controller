#include <state_controller.hpp>

int main(int argc, char **argv) {
  ros::init(argc, argv, "nturt_state_controller_node");

  auto node = std::shared_ptr<ros::NodeHandle>(new ros::NodeHandle());

  TC my_tc(node);

  ros::Rate loop_rate(1000);

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}