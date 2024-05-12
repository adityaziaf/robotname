#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robotname_msgs/action/flush_intake.hpp"
#include "std_msgs/msg/int32.hpp"
#include "behaviortree_ros2/bt_action_node.hpp"

class FlushIntakeServer : public rclcpp::Node
{
public:
  using FlushIntake = robotname_msgs::action::FlushIntake;
  using GoalHandleFlushIntake = rclcpp_action::ServerGoalHandle<FlushIntake>;

  explicit FlushIntakeServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("FlushIntake_action_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<FlushIntake>(
      this,
      "flush_intake",
      std::bind(&FlushIntakeServer::handle_goal, this, _1, _2),
      std::bind(&FlushIntakeServer::handle_cancel, this, _1),
      std::bind(&FlushIntakeServer::handle_accepted, this, _1));
        
      _intake_sub = this->create_subscription<std_msgs::msg::Int32>("/ballcount", 1, std::bind(&FlushIntakeServer::handle_subscription, this,_1));
      
    // 
  }

private:
  rclcpp_action::Server<FlushIntake>::SharedPtr action_server_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr _intake_sub;
  std_msgs::msg::Int32::SharedPtr last_msg;



  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const FlushIntake::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with FlushIntake %d", goal->target_remaining_ball);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFlushIntake> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleFlushIntake> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&FlushIntakeServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleFlushIntake> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(100);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<FlushIntake::Feedback>();
    auto result = std::make_shared<FlushIntake::Result>();

    bool state = true;

    while(rclcpp::ok() && state)
    {
      if (goal_handle->is_canceling())
      {
        result->status = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      if(last_msg)
      {
        if(last_msg->data == goal->target_remaining_ball)
        {
          state = false;
        }
      }
      
      else
      {
        RCLCPP_INFO(this->get_logger(), "Empty Topic Value");
      }
      
      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok())
    {
      result->status = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }

  void handle_subscription(const std_msgs::msg::Int32::SharedPtr msg)
  {
    last_msg = msg;
  }
};  // class FlushIntakeServer


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FlushIntakeServer>();

  rclcpp::spin(node);

  return 0;
}
