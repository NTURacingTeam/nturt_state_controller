/**
 * @file state_controller.hpp
 * @author quantumspawner jet22854111@gmail.com
 * @brief ROS package for controller the state of electrical system.
 */

#ifndef STATE_CONTROLLER_HPP
#define STATE_CONTROLLER_HPP

// std include
#include <bitset>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

// gpio include
#include <wiringPi.h>

// ros include
#include <ros/ros.h>

// ros message include
#include <can_msgs/Frame.h>
#include <std_msgs/Bool.h>

// nturt include
#include <cp_can_id.hpp>
#include <NTURT_CAN_Parser.hpp>

/**
 * @author quantumspawner jet22854111@gmail.com
 * @brief Class for controlling the states of electrical system.
 */
class StateController {
    public:
        StateController(std::shared_ptr<ros::NodeHandle> &_nh);

        /// @brief Function for update state_controller.
        void update();

        /// @brief Function for converting internal state to string.
        std::string get_string();

    private:
        /// @brief Callback function when receiving message form topic "/sent_messages".
        void onCan(const can_msgs::Frame::ConstPtr &_msg);

        /// @brief Timed callback function called after "button_duration_" without trigger the button.
        void button_callback(const ros::TimerEvent &_event);

        /// @brief Function for playing rtd sound.
        void play_rtd_sound();

        /// @brief Pointer to ros node handle.
        std::shared_ptr<ros::NodeHandle> nh_;

        /// @brief Publisher to "/node_state".
        ros::Publisher state_pub_;

        /// @brief Publisher to "/sent_messages".
        ros::Publisher can_pub_;

        /// @brief Subscriber to "/received_messages".
        ros::Subscriber can_sub_;

        /// @brief CAN parser.
        Parser parser_;

        // ready to drive (rtd)
        /// @brief State of the electrical system.
        int check_state_ = 0;

        /// @brief Voltage of the tractive system [V].
        double ts_voltage_ = 0;

        /// @brief RTD button trigger.
        bool rtd_button_trigger_ = false;

        /// @brief Brake pedal trigger.
        bool brake_trigger_ = false;

        /// @brief If rtd had triggered.
        bool rtd_triggered_ = false;

        // shutdown button
        /// @brief Timer for determine the comman of button.
        ros::Timer button_timer_;

        /// @brief How many times the button is pushed in "button_duration_".
        int button_push_times_ = 0;

        /// @brief Time duration during which button trigger is counted as shutdown/reboot command.
        double button_duration_ = 3;
};

#endif //STATE_CONTROLLER_HPP
