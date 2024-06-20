#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robotname_msgs/action/wait_button.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/bool.hpp"

#include "behaviortree_ros2/bt_action_node.hpp"

class WaitButtonServer : public rclcpp::Node
{
public:
  using WaitButton = robotname_msgs::action::WaitButton;
  using GoalHandleWaitButton = rclcpp_action::ServerGoalHandle<WaitButton>;

  explicit WaitButtonServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("WaitButton_action_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<WaitButton>(
      this,
      "wait_button",
      std::bind(&WaitButtonServer::handle_goal, this, _1, _2),
      std::bind(&WaitButtonServer::handle_cancel, this, _1),
      std::bind(&WaitButtonServer::handle_accepted, this, _1));
        
      _intake_button_sub = this->create_subscription<std_msgs::msg::UInt8>("/button_start", 1, std::bind(&WaitButtonServer::handle_subscription, this,_1));
    // 
  }

private:
  rclcpp_action::Server<WaitButton>::SharedPtr action_server_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr _intake_button_sub;
  std_msgs::msg::UInt8::SharedPtr last_msg;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const WaitButton::Goal> goal)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleWaitButton> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleWaitButton> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&WaitButtonServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleWaitButton> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(100);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<WaitButton::Feedback>();
    auto result = std::make_shared<WaitButton::Result>();

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

      std_msgs::msg::Bool buttonstate;
      buttonstate.data = false;

      if(last_msg)
      {
        
        if(last_msg->data == 1)
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

  void handle_subscription(const std_msgs::msg::UInt8::SharedPtr msg)
  {
    last_msg = msg;
  }

};  // class WaitButtonServer


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WaitButtonServer>();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "WaitButton Server Ready");

  rclcpp::spin(node);

  return 0;
}
