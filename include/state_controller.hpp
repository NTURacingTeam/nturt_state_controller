#ifndef STATE_CONTROLLER_HPP
#define STATE_CONTROLLER_HPP

#include <NTURT_CAN_Parser.hpp>
#include <can_msgs/Frame.h>
#include "std_msgs/Bool.h"
#include <cp_can_id.hpp>
#include <memory>
#include <ros/ros.h>
#include <wiringPi.h>

using namespace std;

class State_Controller {
public:
  State_Controller(std::shared_ptr<ros::NodeHandle> &nh);

  void onCan(const can_msgs::Frame::ConstPtr &msg) {
    int id = msg->id;
    int checkSensor;
    // std::cout << "id: " << id << std::endl;
    switch (id)
    {
      /* case _CAN_FB1:
        if (parser_.decode(_CAN_FB1, msg->data) == OK) {
          checkSensor |= 0b0000000001;
          cout<<"received CAN FB1"<<endl;
        }
        break;*/
      case _CAN_FB2:
        if (parser_.decode(_CAN_FB2, msg->data) == OK) {
          checkSensor |= 0b0000000010;
          brake_trigger_ = parser_.get_afd("BMS", "N");
        }
        break;
      case _CAN_RB1:
        if (parser_.decode(_CAN_FB2, msg->data) == OK) {
          checkSensor |= 0b0000000100;
        }
        break;
      /*case _CAN_RB2:
        if (parser_.decode(_CAN_FB2, msg->data) == OK) {
          checkSensor |= 0b0000001000;
        }
        break;*/
      case _CAN_DB1:
        if (parser_.decode(_CAN_DB1, msg->data) == OK)
        {
          checkSensor |= 0b0000010000;
          RTD_ = parser_.get_afd("RTD", "N");
        }
        break;
      /* case _CAN_BMS:
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
        break;*/
      case _CAN_MOV:
        if (parser_.decode(_CAN_MOV, msg->data) == OK)
        {
          checkSensor |= 0b0000100000;
          TS_voltage_ = parser_.get_afd("MOV", "N"); //min 268 V
        }
      case 
    }

    if (checkSensor == 0b0000010010)
    {
      if(TS_voltage_ > 268)
      {
        if (brake_trigger_)
        {
          can_msgs::Frame RTDmsg;
          RTDmsg.header.stamp = ros::Time::now();
          RTDmsg.id = 0x0008A7D0;
          RTDmsg.is_extended = 1;
          RTDmsg.dlc = 8;
          RTDmsg.data = {1, 0, 0, 0, 0, 0, 0, 1};
          can_pub_.publish(RTDmsg);
          if(RTD_)
          {
            std::cout << "RTD" << std::endl;
            // send can message to the controller
            std_msgs::Bool state;
            state.data = 1;
            state_pub_.publish(state);

            for(int i = 0; i < 3; i++)
            {
              digitalWrite(0,HIGH);
              delay(300);
              digitalWrite(0,LOW);
              delay(150);
            }          
          }
        }
      }
    }
    else{
      std_msgs::Bool state;
      state.data = 0;
      state_pub_.publish(state);
    }    
  }
  void checkVCU() {
    can_msgs::Frame vcu1msg;
    vcu1msg.header.stamp = ros::Time::now();
    vcu1msg.id = 0x0008A7D0;
    vcu1msg.is_extended = 1;
    vcu1msg.dlc = 8;
    vcu1msg.data = {1, 0, 0, 0, 0, 0, 0, 0};
    can_pub_.publish(vcu1msg);
    std::cout << "CAN published" << std::endl;
  }

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

State_Controller::State_Controller(std::shared_ptr<ros::NodeHandle> &nh) : nh_(nh) {
  std::cout << "node init" << std::endl;
  parser_.init_parser();
  state_pub_ = nh_->advertise<std_msgs::Bool>("node_state", 5);
  can_pub_ = nh_->advertise<can_msgs::Frame>("sent_messages", 5);
  can_sub_ = nh_->subscribe("received_messages", 5, &State_Controller::onCan, this);
  
  wiringPiSetup();
  pinMode(0,OUTPUT);
}

typedef State_Controller SC;

#endif