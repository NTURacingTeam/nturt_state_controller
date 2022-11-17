#include "state_controller.hpp"

StateController::StateController(std::shared_ptr<ros::NodeHandle> &_nh) :
    nh_(_nh),
    publish_frame_pub_(_nh->advertise<std_msgs::String>("/publish_can_frame", 10)),
    state_pub_(_nh->advertise<std_msgs::Bool>("/node_state", 10)),
    update_data_pub_(_nh->advertise<nturt_ros_interface::UpdateCanData>("/update_can_data", 10)),
    get_data_clt_(_nh->serviceClient<nturt_ros_interface::GetCanData>("/get_can_data")),
    register_clt_(_nh->serviceClient<nturt_ros_interface::RegisterCanNotification>("/register_can_notification")),
    button_timer_(_nh->createTimer(ros::Duration(1.0), &StateController::button_callback, this, true)) {
    
    // register to can parser
    // wait until "/register_can_notification" service is avalible
    if(!ros::service::waitForService("/register_can_notification", 10000)) {
        ROS_FATAL("register to can parser timeout after 10 seconds");
        ros::shutdown();
    }
    // construct register call
    nturt_ros_interface::RegisterCanNotification register_srv;
    register_srv.request.node_name = ros::this_node::getName();
    /*
    data name registering to be notified
    brake_micro -> brake trigger (front box 2)
    ready_to_drive -> rtd button (dashboard)
    wheel_speed_front_left -> data to determine if rear box is working (rear box 1)
    output_voltage -> mcu voltage (mcu_output_voltage)
    */
    register_srv.request.data_name = {
        "brake_micro",
        "ready_to_drive",
        "front_left_wheel_speed",
        "input_voltage"
    };
    // call service
    if(!register_clt_.call(register_srv)) {
        ROS_FATAL("register to can parser failed");
        ros::shutdown();
    }

    // subscribe to the register topic
    notification_sub_ = nh_->subscribe(register_srv.response.topic, 10, &StateController::onNotification, this);

    // initiate wiringpi gpio
    wiringPiSetup();
    pinMode(29, OUTPUT);
}

void StateController::update() {
    /* 
    ready to drive light condition:
        1. rtd had not yet triggered
        2. all state are set
        3. brake is currently actuated
    */
   bool rtd_light = false;
    if(!rtd_triggered_) {
        if(check_state_ == 0b1111 && brake_trigger_) {
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

    // update vcu data
    nturt_ros_interface::UpdateCanData update_msg;
    // vcu light
    update_msg.name = "vcu_light";
    update_msg.data = 1;
    update_data_pub_.publish(update_msg);
    // rtd light
    update_msg.name = "rtd_light";
    update_msg.data = rtd_light;
    update_data_pub_.publish(update_msg);
}

std::string StateController::get_string() const {
    return std::string("state_controller state:") +
        "\n\tmessage in:" +
        "\n\t\tts_voltage: " + std::to_string(ts_voltage_) +
        "\n\t\trtd_button_trigger: " + std::to_string(rtd_button_trigger_) +
        "\n\t\tbrake_trigger: " + std::to_string(brake_trigger_) +
        "\n\tinternal state:" +
        "\n\t\tcheck_state: " +  std::to_string(check_state_) +
        "\n\t\trtd_triggered: " + (rtd_triggered_ ? "true" : "false") +
        "\n\t\tbutton_push_times: " + std::to_string(button_push_times_);
}

void StateController::onNotification(const nturt_ros_interface::UpdateCanData::ConstPtr &_msg) {
    if(_msg->name == "brake_micro") {
        // front box state 1
        check_state_ |= 0b1;
        brake_trigger_ = _msg->data;
    }
    else if(_msg->name == "ready_to_drive") {
        // dashboard state 2
        check_state_ |= 0b10;
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
    }
    else if(_msg->name == "front_left_wheel_speed") {
        // rear box state 4
        check_state_ |= 0b100;
    }
    else if(_msg->name == "input_voltage") {
        ts_voltage_ = _msg->data;
        // minium 268 V
        if(ts_voltage_ >= 268) {
            // mcu state 8
            check_state_ |= 0b1000;
        }
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

void StateController::play_rtd_sound() const{
    for(int i = 0; i < 3; i++) {
        digitalWrite(29, HIGH);
        ros::Duration(0.3).sleep();
        digitalWrite(29, LOW);
        ros::Duration(0.15).sleep();
    }
}
