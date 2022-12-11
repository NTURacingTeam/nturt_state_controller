#include "state_controller.hpp"

StateController::StateController(std::shared_ptr<ros::NodeHandle> &_nh) :
    nh_(_nh),
    publish_frame_pub_(_nh->advertise<std_msgs::String>("/publish_can_frame", 10)),
    state_pub_(_nh->advertise<std_msgs::Bool>("/node_state", 10)),
    update_data_pub_(_nh->advertise<nturt_ros_interface::UpdateCanData>("/update_can_data", 10)),
    get_data_clt_(_nh->serviceClient<nturt_ros_interface::GetCanData>("/get_can_data")),
    register_clt_(_nh->serviceClient<nturt_ros_interface::RegisterCanNotification>("/register_can_notification")),
    button_timer_(_nh->createTimer(ros::Duration(1.0), &StateController::button_callback, this, true)),
    last_stable_time_(state_to_check_.size(), 0) {
    
    // initialize state_to_check_filter_
    for(int i = 0; i < state_to_check_.size(); i++) {
        state_to_check_filter_[state_to_check_[i]] = i;
    }

    // register to can parser
    // wait until "/register_can_notification" service is avalible
    if(!ros::service::waitForService("/register_can_notification", 10000)) {
        ROS_FATAL("register to can parser timeout after 10 seconds");
        ros::shutdown();
    }
    // construct register call
    nturt_ros_interface::RegisterCanNotification register_srv;
    register_srv.request.node_name = ros::this_node::getName();
    register_srv.request.data_name = state_to_check_;
    // call service
    if(!register_clt_.call(register_srv)) {
        ROS_FATAL("register to can parser failed");
        ros::shutdown();
    }

    // subscribe to the register topic
    notification_sub_ = nh_->subscribe(register_srv.response.topic, 1000, &StateController::onNotification, this);

    // initiate wiringpi gpio
    wiringPiSetup();
    pinMode(29, OUTPUT);
}

void StateController::update() {
    bool vcu_light = false;
    bool rtd_light = false;
    bool activate = false;
    
    if(check_state()) {
        if(!rtd_triggered_) {
            if(brake_trigger_) {
                if(rtd_button_trigger_) {
                    rtd_triggered_ = true;
                    std::thread play_rtd_sound_thread(&StateController::play_rtd_sound, this);
                    play_rtd_sound_thread.detach();
                }
                else {
                    rtd_light = true;
                }
            }
            else {
                // blink rtd_light, with frequency of 1 Hz
                double now = ros::Time::now().toSec(), intpart;
                std::modf(now * 2, &intpart);
                rtd_light = static_cast<uint64_t>(intpart) % 2;

            }
        }
        else {
            activate = true;
        }
        vcu_light = true;
    }
    else {
        rtd_triggered_ = false;
        ROS_WARN("Warn: VCU is deactivated by %s.", deactivated_by_.c_str());

        // blink vcu_light, with frequency of 1 Hz
        double now = ros::Time::now().toSec(), intpart;
        std::modf(now * 2, &intpart);
        vcu_light = static_cast<uint64_t>(intpart) % 2;
    }

    // publish state to "/node_state"
    std_msgs::Bool state;
    state.data = activate;
    state_pub_.publish(state);
    
    // update vcu data
    nturt_ros_interface::UpdateCanData update_msg;
    // vcu light
    update_msg.name = "vcu_light";
    update_msg.data = vcu_light;
    update_data_pub_.publish(update_msg);
    // rtd light
    update_msg.name = "rtd_light";
    update_msg.data = rtd_light;
    update_data_pub_.publish(update_msg);
}

std::string StateController::get_string() const {
    std::string last_stable_time;
    for(int i = 0; i < state_to_check_.size(); i++) {
        last_stable_time += (std::string("\n\t\t\t") + state_to_check_[i] + " :" + std::to_string(last_stable_time_[i]));
    }

    return std::string("state_controller state:") +
        "\n\tmessage in:" +
        "\n\t\tts_voltage: " + std::to_string(ts_voltage_) +
        "\n\t\trtd_button_trigger: " + std::to_string(rtd_button_trigger_) +
        "\n\t\tbrake_trigger: " + std::to_string(brake_trigger_) +
        "\n\tinternal state:" +
        "\n\t\tcheck_state: " + (check_state_const() ? "true" : "false") +
        "\n\t\tlast_stable_time: " + last_stable_time +
        "\n\t\trtd_triggered: " + (rtd_triggered_ ? "true" : "false") +
        "\n\t\tbutton_push_times: " + std::to_string(button_push_times_);
}

void StateController::onNotification(const nturt_ros_interface::UpdateCanData::ConstPtr &_msg) {
    switch(state_to_check_filter_[_msg->name]) {
        // break_micro
        case 0:
            brake_trigger_ = _msg->data;
            
            last_stable_time_[0] = ros::Time::now().toSec();
            break;
        
        // ready_to_drive
        case 1:
            rtd_button_trigger_ = _msg->data;
            // button shutdown/reboot control
            if(rtd_button_trigger_) {
                if(button_push_times_ == 0) {
                    button_push_times_++;
                    button_timer_.stop();
                    button_timer_.start();
                }
                else {
                    button_push_times_++;
                    button_timer_.stop();
                    button_timer_.start();
                }
            }

            last_stable_time_[1] = ros::Time::now().toSec();
            break;

        // rear_left_wheel_speed
        case 2:
            last_stable_time_[2] = ros::Time::now().toSec();
            break;
        
        // input_voltage
        case 3:
            ts_voltage_ = _msg->data;

            // minium 250 V
            if(ts_voltage_ >= 250) {
                last_stable_time_[3] = ros::Time::now().toSec();
            }
            break;
    }
}

void StateController::button_callback(const ros::TimerEvent &_event) {
    if(button_push_times_ >= 5) {
        if(system("echo \"sudo service nturt_ros stop && sudo poweroff\" > $(rospack find nturt_rpi_deployer)/nturt_ros_pipe") != 0) {
            ROS_FATAL("Failed to execute shutdown command.");
        }
        button_push_times_ = 0;
    }
    else if(button_push_times_ >= 3) {
        if(system("echo \"sudo service nturt_ros stop && sudo reboot\" > $(rospack find nturt_rpi_deployer)/nturt_ros_pipe") != 0) {
            ROS_FATAL("Failed to execute reboot command.");
        }
        button_push_times_ = 0;
    }
    else {
        button_push_times_ = 0;
    }
}

bool StateController::check_state() {
    double current_time = ros::Time::now().toSec();
    for(int i = 0; i < state_to_check_.size(); i++) {
        if(current_time - last_stable_time_[i] > deactive_threshold_) {
            deactivated_by_ = state_to_check_[i];
            return false;
        }
    }

    return true;
}

bool StateController::check_state_const() const {
    double current_time = ros::Time::now().toSec();
    for(const double &last_time : last_stable_time_) {
        if(current_time - last_time > deactive_threshold_) {
            return false;
        }
    }

    return true;
}

void StateController::play_rtd_sound() const{
    for(int i = 0; i < 3; i++) {
        digitalWrite(29, HIGH);
        ros::Duration(0.3).sleep();
        digitalWrite(29, LOW);
        ros::Duration(0.15).sleep();
    }
}
