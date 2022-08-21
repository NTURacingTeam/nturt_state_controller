#include <state_controller.hpp>

int main(int argc, char **argv) {
    ros::init(argc, argv, "nturt_state_controller_node");

    auto nh = std::shared_ptr<ros::NodeHandle>(new ros::NodeHandle());

    StateController state_controller(nh);

    ros::Rate loop_rate(10);

    while (ros::ok()) {
        ros::spinOnce();
        state_controller.checkVCU();
        loop_rate.sleep();
    }

    return 0;
}
