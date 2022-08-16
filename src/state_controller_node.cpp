#include <state_controller.hpp>

int main(int argc, char **argv) {
  ros::init(argc, argv, "nturt_state_controller_node");

  auto node = std::shared_ptr<ros::NodeHandle>(new ros::NodeHandle());

  SC my_sc(node);

  ros::Rate loop_rate(100);

  while (ros::ok()) {
    ros::spinOnce();
    my_sc.checkVCU();
    loop_rate.sleep();
  }

  return 0;
}