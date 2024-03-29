/**
 * @file state_controller.hpp
 * @author quantumspawner jet22854111@gmail.com
 * @brief ROS package for controlling the state of other nodes in nturt.
 */

#ifndef STATE_CONTROLLER_HPP
#define STATE_CONTROLLER_HPP

// std include
#include <map>
#include <math.h>
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
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief Class for controlling the states of other nodes in nturt.
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

        /// @brief Timer for determine the comman of button.
        ros::Timer button_timer_;

        // internal control parameters
        // ready to drive (rtd)
        /**
         * @brief State of the vehicle to be checked, which is determined by can data name corresponds to that state.
         * 
         * brake_micro -> brake trigger (front_box_2)
         * ready_to_drive -> rtd button (dashboard)
         * front_left_wheel_speed -> data to determine if rear box is working (rear_box_1)
         * input_voltage -> mcu voltage (mcu_voltage)
         */
        const std::vector<std::string> state_to_check_ = {
            "brake_micro",
            "ready_to_drive",
            "rear_left_wheel_speed",
            "input_voltage"
        };

        /// @brief Vector storing last stable time of the vehicle.
        std::vector<double> last_stable_time_;
        
        /// @brief Map mapping the state of the vehicle to be checked to its index in "check_state_".
        std::map<std::string, int> state_to_check_filter_;

        /// @brief Voltage of the tractive system \f$[V]\f$.
        double ts_voltage_ = 0;

        /// @brief RTD button trigger.
        bool rtd_button_trigger_ = false;

        /// @brief Brake pedal trigger.
        bool brake_trigger_ = false;

        /// @brief If rtd had triggered.
        bool rtd_triggered_ = false;

        /// @brief Time threshold when time difference between "last_stable_time" and present time is higher, deactive the vcu \f$[s]\f$.
        double deactive_threshold_ = 2.0;
        
        /// @brief State that vcu is deactivated by, which is determined by can data name corresponds to that state.
        std::string deactivated_by_ = "";

        // shutdown button
        /// @brief How many times the button is pushed in "button_duration_".
        int button_push_times_ = 0;

        /// @brief Time duration during which button trigger is counted as shutdown/reboot command.
        const double button_duration_ = 3;

        /// @brief Callback function when receiving message form can data notification.
        void onNotification(const nturt_ros_interface::UpdateCanData::ConstPtr &_msg);

        /// @brief Timed callback function called after "button_duration_" without trigger the button.
        void button_callback(const ros::TimerEvent &_event);

        /**
         * @brief Function to check the state of the vehicle, modify "deactivated_by_" to the state that is not good.
         * @return true If all state of the vehicle are good.
         */
        bool check_state();

        /**
         * @brief Function to check the state of the vehicle.
         * @return true If all state of the vehicle are good.
         */
        bool check_state_const() const;

        /// @brief Function for playing rtd sound.
        void play_rtd_sound() const;
};

#endif //STATE_CONTROLLER_HPP
