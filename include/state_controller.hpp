#ifndef STATE_CONTROLLER_HPP
#define STATE_CONTROLLER_HPP

// std include
#include <bitset>
#include <iostream>
#include <memory>
#include <string>

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

        /// @brief Timed callback function called after "button_duration_" without trigger the button
        void button_callback(const ros::TimerEvent &_event);

        Parser parser_;
        std::shared_ptr<ros::NodeHandle> nh_;
        ros::Publisher state_pub_;
        ros::Publisher can_pub_;
        ros::Subscriber can_sub_;
        int check_sensor_ = 0;
        double TS_voltage_ = 0;
        bool brake_trigger_ = false;
        bool RTD_ = false;

        // shutdown button
        /// @brief Timer for determine the comman of button
        ros::Timer button_timer_;

        /// @brief How many times the button is pushed in "button_duration_"
        int button_push_times_ = 0;

        /// @brief Time duration during which button trigger is counted as shutdown/reboot command
        double button_duration_ = 3;
};

#endif //STATE_CONTROLLER_HPP
