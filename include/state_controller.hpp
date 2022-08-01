#ifndef STATE_CONTROLLER_HPP
#define STATE_CONTROLLER_HPP

#include <NTURT_CAN_Parser.hpp>
#include <can_msgs/Frame.h>
#include "std_msgs/Bool.h"
#include <cp_can_id.hpp>
#include <memory>
#include <ros/ros.h>

class State_Controller {
public:
  State_Controller(std::shared_ptr<ros::NodeHandle> &nh);

  void onCan(const can_msgs::Frame::ConstPtr &msg) {
    int id = msg->id;
    int checkSensor;
    // std::cout << "id: " << id << std::endl;
    switch (id)
    {
      case _CAN_FB1:
        if (parser_.decode(_CAN_FB1, msg->data) == OK) {
          checkSensor |= 0b0000000001;
        }
        break;
      case _CAN_FB2:
        if (parser_.decode(_CAN_FB2, msg->data) == OK) {
          checkSensor |= 0b0000000010;
        }
        break;
      case _CAN_RB1:
        if (parser_.decode(_CAN_FB2, msg->data) == OK) {
          checkSensor |= 0b0000000100;
        }
        break;
      case _CAN_RB2:
        if (parser_.decode(_CAN_FB2, msg->data) == OK) {
          checkSensor |= 0b0000001000;
        }
        break;
      case _CAN_DB1:
        if (parser_.decode(_CAN_FB2, msg->data) == OK) {
          checkSensor |= 0b0000010000;
        }
        break;
      case _CAN_DB2:
        if (parser_.decode(_CAN_FB2, msg->data) == OK) {
          checkSensor |= 0b0000100000;
        }
        break;
      case _CAN_BMS:
        if (parser_.decode(_CAN_FB2, msg->data) == OK) {
          checkSensor |= 0b0001000000;
        }
        break;
      case _CAN_HIS:
        if (parser_.decode(_CAN_FB2, msg->data) == OK) {
          checkSensor |= 0b0010000000;
        }
        break;
      case _CAN_HIA:
        if (parser_.decode(_CAN_FB2, msg->data) == OK) {
          checkSensor |= 0b0100000000;
        }
        break;
      case _CAN_HIG:
        if (parser_.decode(_CAN_FB2, msg->data) == OK) {
          checkSensor |= 0b1000000000;
        }
        break;
    }

    if (checkSensor == 0b1111111111){
      // send can message to the controller
      std_msgs::Bool state;
      state.data = 1;
      state_pub_.publish(state);
    }
    else{
      std_msgs::Bool state;
      state.data = 0;
      state_pub_.publish(state);
    }    
  }

private:
  Parser parser_;
  std::shared_ptr<ros::NodeHandle> nh_;
  ros::Publisher state_pub_;
  ros::Subscriber can_sub_;
};

State_Controller::State_Controller(std::shared_ptr<ros::NodeHandle> &nh) : nh_(nh) {
  std::cout << "node init" << std::endl;
  state_pub_ = nh_->advertise<std_msgs::Bool>("node_state", 5);
  can_sub_ = nh_->subscribe("received_messages", 5, &State_Controller::onCan, this);
}

typedef State_Controller SC;

#endif