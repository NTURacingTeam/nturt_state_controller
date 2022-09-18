#include "state_controller.hpp"

int main(int argc, char **argv) {
    // register as a ros node
    ros::init(argc, argv, "nturt_state_controller_node");

    // create a node handle
    auto nh = std::shared_ptr<ros::NodeHandle>(new ros::NodeHandle());

    // initialize state controller
    StateController state_controller(nh);

    // frequancy 10 Hz
    ros::Rate loop_rate(10);

    while (ros::ok()) {
        ros::spinOnce();
        state_controller.update();
        loop_rate.sleep();
    }

    return 0;
}
