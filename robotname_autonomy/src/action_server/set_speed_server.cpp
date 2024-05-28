#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robotname_msgs/action/set_speed_server.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "behaviortree_ros2/bt_action_node.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "robotname_msgs/msg/detection_array.hpp"
#include "tf2/utils.h"

class SetSpeedServer : public rclcpp::Node
{
public:
  using SetSpeedMove = robotname_msgs::action::SetSpeedServer;
  using GoalHandleSetSpeed = rclcpp_action::ServerGoalHandle<SetSpeedMove>;

  explicit SetSpeedServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("SetSpeed_action_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<SetSpeedMove>(
      this,
      "set_speed",
      std::bind(&SetSpeedServer::handle_goal, this, _1, _2),
      std::bind(&SetSpeedServer::handle_cancel, this, _1),
      std::bind(&SetSpeedServer::handle_accepted, this, _1));
    
    twist_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",10);
  }

private:
  rclcpp_action::Server<SetSpeedMove>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub;
  rclcpp::TimerBase::SharedPtr timer_;
 

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const SetSpeedMove::Goal> goal)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleSetSpeed> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleSetSpeed> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&SetSpeedServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleSetSpeed> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(100);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<SetSpeedMove::Feedback>();
    auto result = std::make_shared<SetSpeedMove::Result>();

    rclcpp::Time deadline = get_clock()->now() + rclcpp::Duration::from_seconds( double(goal->timeout) / 1000 );

    geometry_msgs::msg::Twist tw_msg;

    while( get_clock()->now() < deadline && rclcpp::ok())
    {
      if (goal_handle->is_canceling())
      {
        result->status = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      
      tw_msg.linear.x = goal->speed_x;
      tw_msg.linear.y = goal->speed_y;
      tw_msg.angular.z = goal->theta;

      //feedback->current_readings = cycle++;
      //goal_handle->publish_feedback(feedback);
      //RCLCPP_INFO(this->get_logger(), "Publish feedback");
      twist_pub->publish(tw_msg);

      loop_rate.sleep();
    }
    tw_msg.linear.x = 0.0;
    tw_msg.linear.y = 0.0;
    tw_msg.angular.z = 0.0;

    twist_pub->publish(tw_msg);
    // Check if goal is done
    if (rclcpp::ok())
    {
      result->status = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
};  // class SetSpeedServer


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SetSpeedServer>();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Set Speed Server Ready");
  rclcpp::spin(node);

  return 0;
}
