#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "robotname_msgs/action/navigate.hpp"
#include "robotname_navigation/pid.hpp"
#include "robotname_navigation/visibility_control.h"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace robotname_navigation {
class NavigateActionServer : public rclcpp::Node {
 public:
  using Navigate = robotname_msgs::action::Navigate;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Navigate>;

  ROBOTNAME_NAVIGATION_PUBLIC
  explicit NavigateActionServer(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node("fibonacci_action_server", options) {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Navigate>(
        this, "navigate",
        std::bind(&NavigateActionServer::handle_goal, this, _1, _2),
        std::bind(&NavigateActionServer::handle_cancel, this, _1),
        std::bind(&NavigateActionServer::handle_accepted, this, _1));
    
    //cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);
    //odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom",10, std::bind(&NavigateActionServer::callbackOdom, this, _1));
  }

 private:
  rclcpp_action::Server<Navigate>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const Navigate::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received goal request with order %f",
                goal->target_distance);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandle> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a
    // new thread
    std::thread{std::bind(&NavigateActionServer::execute, this, _1),
                goal_handle}
        .detach();
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Navigate::Feedback>();
    auto result = std::make_shared<Navigate::Result>();
    float curr_distance = 5.4;
    PID pid = PID(0.5, 10, -10, 1, 0.01, 1);


    while (curr_distance <= goal->target_distance && rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        result->error_code = 0;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      double inc = pid.calculate(goal->target_distance, curr_distance);

      //geometry_msgs::msg::Twist msg;
      //msg.linear.x = inc;

      //cmd_pub_->publish(msg);
      curr_distance += 0.1;

      feedback->remaining_distance = goal->target_distance - curr_distance;
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");
      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->error_code = 1;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
  
//   nav_msgs::msg::Odometry::SharedPtr latest_odom_msg;
//   void callbackOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
//   {
//     latest_odom_msg = msg;
//   }
};  // class FibonacciActionServer

}  // namespace robotname_navigation

RCLCPP_COMPONENTS_REGISTER_NODE(robotname_navigation::NavigateActionServer)