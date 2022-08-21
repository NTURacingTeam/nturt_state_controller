#ifndef STATE_CONTROLLER_HPP
#define STATE_CONTROLLER_HPP

// std include
#include <iostream>
#include <memory>

// ros include
#include <ros/ros.h>

// ros message include
#include <can_msgs/Frame.h>
#include <std_msgs/Bool.h>

// nturt include
#include <cp_can_id.hpp>
#include <NTURT_CAN_Parser.hpp>

#include <wiringPi.h>

class StateController {
    public:
        StateController(std::shared_ptr<ros::NodeHandle> &_nh);

        void onCan(const can_msgs::Frame::ConstPtr &_msg);
        
        void checkVCU();

    private:
        Parser parser_;
        std::shared_ptr<ros::NodeHandle> nh_;
        ros::Publisher state_pub_;
        ros::Publisher can_pub_;
        ros::Subscriber can_sub_;
        double TS_voltage_ = 0;
        bool brake_trigger_ = false;
        bool RTD_ = false;
};

#endif //STATE_CONTROLLER_HPP
