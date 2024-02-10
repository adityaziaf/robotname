
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robotname_navigation/pid.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int16.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class SimplePlanner : public rclcpp::Node {
 public:
  SimplePlanner() : Node("simpleplanner") {
    timer_ = this->create_wall_timer(
        500ms, std::bind(&SimplePlanner::timerCallback, this));
    status_publisher_ =
        this->create_publisher<std_msgs::msg::Int16>("planner/status", 10);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10,
        std::bind(&SimplePlanner::odomCallback, this, std::placeholders::_1));
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    target_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "planner/target", 10,
        std::bind(&SimplePlanner::targetCallback, this, std::placeholders::_1));
    pid = std::make_unique<PID>(0.1, 1, 0, 0.5, 0.01, 0.5);
  }

 private:
  nav_msgs::msg::Odometry::SharedPtr latest_odom_msg;
  void odomCallback(nav_msgs::msg::Odometry::SharedPtr msg) {
    curr_pos = msg->pose.pose.position.x;
  }

  void timerCallback() {
    std_msgs::msg::Int16 msg_status;

    if (curr_pos <= target_pos) {
      double inc = pid->calculate(target_pos, curr_pos);
      geometry_msgs::msg::Twist msg_twist;
      msg_twist.linear.x = inc;
      cmd_pub_->publish(msg_twist);
      msg_status.data = 1;
    }
    else {msg_status.data = 0; }
    status_publisher_->publish(msg_status);
  }

  std_msgs::msg::Float32::SharedPtr latest_target;
  void targetCallback(std_msgs::msg::Float32::SharedPtr msg) {
    target_pos = msg->data;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr status_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr target_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  std::unique_ptr<PID> pid;
  double curr_pos, target_pos;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimplePlanner>());
  rclcpp::shutdown();
  return 0;
}
