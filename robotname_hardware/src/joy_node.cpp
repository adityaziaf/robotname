
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joy_feedback.hpp"
#include <iostream>
#include <chrono>
#include <stdio.h>
#include "robotname_hardware/remote-controller/ds4.h"

using namespace std::chrono_literals;

class JoyNode : public rclcpp::Node {
private:
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr publisher;
    rclcpp::Subscription<sensor_msgs::msg::JoyFeedback>::SharedPtr subscriptor;
    rclcpp::TimerBase::SharedPtr timer;
    Joystick joy = Joystick("/dev/input/js1");
    sensor_msgs::msg::Joy joy_msg = sensor_msgs::msg::Joy();

    void mainLoop_callback() {
        // std::cout << joy_msg.buttons.size() << std::endl;
        joy.event();
        auto val = sensor_msgs::msg::Joy();
        // val.button = joy.data.button;
        // val.lx = joy.data.lx;
        // val.ly = -joy.data.ly;
        // val.rx = joy.data.rx;
        // val.ry = -joy.data.ry;
        // val.l2 = joy.data.l2;
        // val.r2 = joy.data.r2;

        val.header.frame_id = "joy";

        val.buttons.push_back(joy.data.button);
        val.axes = {joy.data.lx/(float)128.0,
                    joy.data.ly/(float)128.0,
                    joy.data.l2/(float)256.0,
                    joy.data.rx/(float)128.0,
                    joy.data.ry/(float)128.0,
                    joy.data.r2/(float)256.0};

        this->publisher->publish(val);
    };

public:
    JoyNode() : Node("joy_node") {
        // joy_msg.buttons.push_back(0);
        timer = this->create_wall_timer(1000us, std::bind(&JoyNode::mainLoop_callback, this));
        publisher = this->create_publisher<sensor_msgs::msg::Joy>("joy", 10);
        subscriptor  = this->create_subscription<sensor_msgs::msg::JoyFeedback> (
                      "joy/feedback", 10, std::bind(&JoyNode::feedbackCallback, this, std::placeholders::_1));
    };

    void feedbackCallback(const sensor_msgs::msg::JoyFeedback &msg) {
        if(msg.type == msg.TYPE_LED) {
            uint32_t temp = *(uint32_t*)&msg.intensity;
            joy.change_led(temp);
            std::cout << temp << std::endl;
        }
        else if(msg.type == msg.TYPE_RUMBLE) {

        }
        else if(msg.type == msg.TYPE_BUZZER) {

        }
        else {
            std::cout << "Type not found" << std::endl;
        }
    };
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
