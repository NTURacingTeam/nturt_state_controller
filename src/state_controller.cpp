#include "state_controller.hpp"

StateController::StateController(std::shared_ptr<ros::NodeHandle> &_nh) :
    nh_(_nh), 
    state_pub_(nh_->advertise<std_msgs::Bool>("/node_state", 10)),
    can_pub_(nh_->advertise<can_msgs::Frame>("/sent_messages", 10)),
    can_sub_(nh_->subscribe("/received_messages", 10, &StateController::onCan, this)),
    button_timer_(_nh->createTimer(ros::Duration(1.0), &StateController::button_callback, this, true)) {
    
    parser_.init_parser();

    #ifdef __arm__
    wiringPiSetup();
    pinMode(1, OUTPUT);
    #endif
}

void StateController::update() {
    bool rtd_light = false;

    /* ready to drive light condition:
        1. rtd had not yet triggered
        2. all state are set
        3. brake is currently actuated
    */
    if(!rtd_triggered_) {
        if(check_state_ == 0b0111 && brake_trigger_) {
            rtd_light = true;
            if(rtd_button_trigger_) {
                rtd_triggered_ = true;
                std::thread th(&StateController::play_rtd_sound, this);
                th.detach();
            }
        }
        std_msgs::Bool state;
        state.data = false;
        state_pub_.publish(state);
    }
    else {
        std_msgs::Bool state;
        state.data = true;
        state_pub_.publish(state);
    }

    // vcu can signal
    can_msgs::Frame vcu_msg;
    vcu_msg.header.stamp = ros::Time::now();
    vcu_msg.id = 0x0008A7D0;
    vcu_msg.is_extended = 1;
    vcu_msg.dlc = 8;
    vcu_msg.data = {1, 0, 0, 0, 0, 0, 0, rtd_light};
    can_pub_.publish(vcu_msg);
}

std::string StateController::get_string() {
    return std::string("state_controller state:") +
        "\n\tmessage in:" +
        "\n\t\tts_voltage: " + std::to_string(ts_voltage_) +
        "\n\t\trtd_button_trigger: " + std::to_string(rtd_button_trigger_) +
        "\n\t\tbrake_trigger: " + std::to_string(brake_trigger_) +
        "\n\tinternal state:" +
        "\n\t\tcheck_state: " +  std::to_string(check_state_) +
        "\n\t\trtd_triggered: " + (rtd_triggered_ ? "true" : "false") +
        "\n\t\tbutton_push_times: " + std::to_string(button_push_times_) + '\n';
}

void StateController::onCan(const can_msgs::Frame::ConstPtr &_msg) {
    switch(_msg->id) {
        case _CAN_FB2:
            if(parser_.decode(_CAN_FB2, _msg->data) == OK) {
                // front box state 1
                check_state_ |= 0b1;
                brake_trigger_ = std::bitset<8>(_msg->data[7])[1];
            }
            break;

        case _CAN_RB1:
            if(parser_.decode(_CAN_RB1, _msg->data) == OK) {
                // front box state 2
                check_state_ |= 0b10;
            }
            break;

        case _CAN_DB1:
            if(parser_.decode(_CAN_DB1, _msg->data) == OK) {
                // front box state 4
                check_state_ |= 0b100;
                rtd_button_trigger_ = _msg->data[0];
                
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
            }
            break;

        case _CAN_MOV:
            if(parser_.decode(_CAN_MOV, _msg->data) == OK) {
                // front box state 8
                check_state_ |= 0b1000;
                ts_voltage_ = parser_.get_afd("MOV", "N"); //min 268 V
            }
            break;
    }
}

void StateController::button_callback(const ros::TimerEvent &_event) {
    if(button_push_times_ >= 5) {
        system("echo \"sudo service nturt_ros stop && sudo poweroff\" > $(rospack find nturt_rpi_deployer)/nturt_ros_pipe");
        button_push_times_ = 0;
    }
    else if(button_push_times_ >= 3) {
        system("echo \"sudo service nturt_ros stop && sudo reboot\" > $(rospack find nturt_rpi_deployer)/nturt_ros_pipe");
        button_push_times_ = 0;
    }
    else {
        button_push_times_ = 0;
    }
}

void StateController::play_rtd_sound() {
    #ifdef __arm__
    for(int i = 0; i < 3; i++) {
        digitalWrite(1, HIGH);
        delay(300);
        digitalWrite(1, LOW);
        delay(150);
    }
    #endif
}
