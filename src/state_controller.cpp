#include "state_controller.hpp"

StateController::StateController(std::shared_ptr<ros::NodeHandle> &_nh) : nh_(_nh), 
                                 state_pub_(nh_->advertise<std_msgs::Bool>("node_state", 5)),
                                 can_pub_(nh_->advertise<can_msgs::Frame>("sent_messages", 5)),
                                 can_sub_(nh_->subscribe("received_messages", 5, &StateController::onCan, this)),
                                 button_timer_(_nh->createTimer(ros::Duration(1.0), &StateController::button_callback, this, true))
{
    ROS_INFO("node init");
    
    parser_.init_parser();
    wiringPiSetup();
    pinMode(1, OUTPUT);
}

void StateController::onCan(const can_msgs::Frame::ConstPtr &_msg)
{
    int id = _msg->id;
    switch (id)
    {
    case _CAN_FB2:
        if (parser_.decode(_CAN_FB2, _msg->data) == OK)
        {
            check_sensor_ |= 0b0001;
            brake_trigger_ = std::bitset<8>(_msg->data[7])[1];
            ROS_INFO("get FB");
        }
        break;

    case _CAN_RB1:
        if (parser_.decode(_CAN_RB1, _msg->data) == OK)
        {
            // check_sensor_ |= 0b0010;
            ROS_INFO("get RB");
        }
        break;

    case _CAN_DB1:
        if (parser_.decode(_CAN_DB1, _msg->data) == OK)
        {
            // check_sensor_ |= 0b0100;
            ROS_INFO("get DB");
            RTD_ = _msg->data[0];

            // button shutdown/reboot control
            if (button_push_times_ == 0)
            {
                button_push_times_++;
                button_timer_.start();
            }
            else
            {
                button_push_times_++;
                button_timer_.stop();
                button_timer_.start();
            }
        }
        break;

        // case _CAN_MOV:
        //     if (parser_.decode(_CAN_MOV, _msg->data) == OK) {
        //         check_sensor_ |= 0b1000;
        //         TS_voltage_ = parser_.get_afd("MOV", "N"); //min 268 V
        //     }
    }

    if (check_sensor_ == 0b0001)
    {
        // if(TS_voltage_ > 268) {
        if (brake_trigger_)
        {
            can_msgs::Frame RTDmsg;
            RTDmsg.header.stamp = ros::Time::now();
            RTDmsg.id = 0x0008A7D0;
            RTDmsg.is_extended = 1;
            RTDmsg.dlc = 8;
            RTDmsg.data = {1, 0, 0, 0, 0, 0, 0, 1};
            can_pub_.publish(RTDmsg);
            if (RTD_)
            {
                ROS_INFO("RTD");
                // send can message to the controller
                std_msgs::Bool state;

                state.data = true;
                state_pub_.publish(state);

                for (int i = 0; i < 3; i++)
                {
                    digitalWrite(1, HIGH);
                    delay(300);
                    digitalWrite(1, LOW);
                    delay(150);
                }
            }
        }
        // }
    }
    else
    {
        std_msgs::Bool state;
        state.data = 0;
        state_pub_.publish(state);
    }

    ROS_WARN("Check sensor: %d", check_sensor_);
}

void StateController::checkVCU()
{
    can_msgs::Frame vcu1msg;
    vcu1msg.header.stamp = ros::Time::now();
    vcu1msg.id = 0x0008A7D0;
    vcu1msg.is_extended = 1;
    vcu1msg.dlc = 8;
    vcu1msg.data = {1, 0, 0, 0, 0, 0, 0, 0};
    can_pub_.publish(vcu1msg);
    ROS_INFO("VCU published");
}

void StateController::button_callback(const ros::TimerEvent &_event)
{
    if (button_push_times_ >= 5)
    {
        system("echo sudo service nturt_ros stop > $(rospack find nturt_deploy_to_rpi)/nturt_ros_pipe");
        system("echo sudo poweroff > $(rospack find nturt_deploy_to_rpi)/nturt_ros_pipe");
        button_push_times_ = 0;
    }
    else if (button_push_times_ >= 3)
    {
        system("echo sudo service nturt_ros stop > $(rospack find nturt_deploy_to_rpi)/nturt_ros_pipe");
        system("echo sudo reboot > $(rospack find nturt_deploy_to_rpi)/nturt_ros_pipe");
        button_push_times_ = 0;
    }
    else
    {
        button_push_times_ = 0;
    }
}
