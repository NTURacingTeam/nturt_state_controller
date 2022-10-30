/**
 * @file state_controller.hpp
 * @brief ROS package for controller the state of other nodes in nturt.
 * @author quantumspawner jet22854111@gmail.com
 */

#ifndef STATE_CONTROLLER_HPP
#define STATE_CONTROLLER_HPP

// std include
#include <memory>
#include <string>
#include <thread>
#include <vector>

// gpio include
#include <wiringPi.h>

// ros include
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

// nturt include
#include "nturt_ros_interface/GetCanData.h"
#include "nturt_ros_interface/RegisterCanNotification.h"
#include "nturt_ros_interface/UpdateCanData.h"

/**
 * @brief Class for controlling the states of other nodes in nturt.
 * @author QuantumSpawner jet22854111@gmail.com
 */
class StateController {
    public:
        /**
         * @brief Constructer.
         * @param _nh Pointer to ros node handle.
         */
        StateController(std::shared_ptr<ros::NodeHandle> &_nh);

        /// @brief Function that should be called every time to update the state controller.
        void update();

        /// @brief Get the string representation of the internal states.
        std::string get_string() const;

    private:
        /// @brief Pointer to ros node handle.
        std::shared_ptr<ros::NodeHandle> nh_;

        /// @brief Publisher to "/publish_can_frame", for publishing can frames (not used for now).
        ros::Publisher publish_frame_pub_;

        /// @brief Publisher to "/node_state", for controlling other nodes in nturt.
        ros::Publisher state_pub_;

        /// @brief Publisher to "/update_can_data", for updating can data.
        ros::Publisher update_data_pub_;

        /// @brief Subscriber to can data notification topic, for getting can data when they got updated.
        ros::Subscriber notification_sub_;
        
        /// @brief Service client to "/get_can_data", for getting can data (not used for now).
        ros::ServiceClient get_data_clt_;

        /// @brief Service client to "/register_can_notification", for registering to notification.
        ros::ServiceClient register_clt_;

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

        /// @brief Callback function when receiving message form can data notification.
        void onNotification(const nturt_ros_interface::UpdateCanData::ConstPtr &_msg);

        /// @brief Timed callback function called after "button_duration_" without trigger the button.
        void button_callback(const ros::TimerEvent &_event);

        /// @brief Function for playing rtd sound.
        void play_rtd_sound() const;
};

#endif //STATE_CONTROLLER_HPP
