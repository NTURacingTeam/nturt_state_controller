#include "state_controller.hpp"

StateController::StateController(std::shared_ptr<ros::NodeHandle> &_nh) : nh_(_nh) {
    std::cout << "node init" << std::endl;
    parser_.init_parser();
    state_pub_ = nh_->advertise<std_msgs::Bool>("node_state", 5);
    can_pub_ = nh_->advertise<can_msgs::Frame>("sent_messages", 5);
    can_sub_ = nh_->subscribe("received_messages", 5, &StateController::onCan, this);
    
    last_time_(ros::Time::now().toSec());
    wiringPiSetup();
    pinMode(1, OUTPUT);
}

void StateController::onCan(const can_msgs::Frame::ConstPtr &_msg) {
    int id = _msg->id;
    int checkSensor;
    // std::cout << "id: " << id << std::endl;
    switch (id) {
        case _CAN_FB2:
            if (parser_.decode(_CAN_FB2, _msg->data) == OK) {
                checkSensor |= 0b0001;
                brake_trigger_ = parser_.get_afd("BMS", "N");
            }
            break;

        case _CAN_RB1:
            if (parser_.decode(_CAN_FB2, _msg->data) == OK) {
                checkSensor |= 0b0010;
            }
            break;

        case _CAN_DB1:
            if (parser_.decode(_CAN_DB1, _msg->data) == OK) {
                checkSensor |= 0b0100;
                RTD_ = parser_.get_afd("RTD", "N");

                // button shutdown/reboot control
                if(button_push_times_ == 0) {
                    button_trigger_time_ = ros::Time::now().toSec();
                    button_push_times_ ++;
                }
                else if(ros::Time::now.toSec() - button_trigger_time_ < button_duration_) {
                    button_push_times_ ++;
                }
                else {
                    if(button_push_times_ >= 5) {
                        system("echo sudo service nturt_ros stop > $(rospack find nturt_deploy_to_rpi)/nturt_ros_pipe");
                        system("echo sudo poweroff > $(rospack find nturt_deploy_to_rpi)/nturt_ros_pipe");
                        button_push_times_ = 0;
                    }
                    else if(button_push_times_ >= 3) {
                        system("echo sudo service nturt_ros stop > $(rospack find nturt_deploy_to_rpi)/nturt_ros_pipe");
                        system("echo sudo reboot > $(rospack find nturt_deploy_to_rpi)/nturt_ros_pipe");
                        button_push_times_ = 0;
                    }
                    else{
                        button_push_times_ = 0;
                    }
                }
            }
            break;

        // case _CAN_MOV:
        //     if (parser_.decode(_CAN_MOV, _msg->data) == OK) {
        //         checkSensor |= 0b1000;
        //         TS_voltage_ = parser_.get_afd("MOV", "N"); //min 268 V
        //     }
    }

    if (checkSensor == 0b0000010010) {
        // if(TS_voltage_ > 268) {
            if (brake_trigger_) {
                can_msgs::Frame RTDmsg;
                RTDmsg.header.stamp = ros::Time::now();
                RTDmsg.id = 0x0008A7D0;
                RTDmsg.is_extended = 1;
                RTDmsg.dlc = 8;
                RTDmsg.data = {1, 0, 0, 0, 0, 0, 0, 1};
                can_pub_.publish(RTDmsg);
                if(RTD_) {
                    std::cout << "RTD" << std::endl;
                    // send can message to the controller
                    std_msgs::Bool state;
                    state.data = 1;
                    state_pub_.publish(state);

                    for(int i = 0; i < 3; i++) {
                        digitalWrite(0,HIGH);
                        delay(300);
                        digitalWrite(0,LOW);
                        delay(150);
                    }
                }
            }
        // }
    }
    else {
        std_msgs::Bool state;
        state.data = 0;
        state_pub_.publish(state);
    }    
}

void StateController::checkVCU() {
    can_msgs::Frame vcu1msg;
    vcu1msg.header.stamp = ros::Time::now();
    vcu1msg.id = 0x0008A7D0;
    vcu1msg.is_extended = 1;
    vcu1msg.dlc = 8;
    vcu1msg.data = {1, 0, 0, 0, 0, 0, 0, 0};
    can_pub_.publish(vcu1msg);
    std::cout << "CAN published" << std::endl;
}
