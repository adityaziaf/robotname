#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robotname_msgs/action/get_intake_color.hpp"
#include "std_msgs/msg/string.hpp"
#include "behaviortree_ros2/bt_action_node.hpp"

class GetIntakeColorServer : public rclcpp::Node
{
public:
  using GetIntakeColor = robotname_msgs::action::GetIntakeColor;
  using GoalHandleGetIntakeColor = rclcpp_action::ServerGoalHandle<GetIntakeColor>;

  explicit GetIntakeColorServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("GetIntakeColor_action_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<GetIntakeColor>(
      this,
      "get_intake_color",
      std::bind(&GetIntakeColorServer::handle_goal, this, _1, _2),
      std::bind(&GetIntakeColorServer::handle_cancel, this, _1),
      std::bind(&GetIntakeColorServer::handle_accepted, this, _1));

    _intake_sub = this->create_subscription<std_msgs::msg::String>("/intake/detectedcolor", 1, std::bind(&GetIntakeColorServer::handle_subscription, this,_1));
  }

private:
  rclcpp_action::Server<GetIntakeColor>::SharedPtr action_server_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _intake_sub;
  std_msgs::msg::String::SharedPtr last_msg;


  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const GetIntakeColor::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with GetIntakeColor %s", goal->color.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleGetIntakeColor> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleGetIntakeColor> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&GetIntakeColorServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleGetIntakeColor> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(100);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<GetIntakeColor::Feedback>();
    auto result = std::make_shared<GetIntakeColor::Result>();

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

      if(!last_msg->data.empty())
      {
        feedback->current_color = last_msg->data;
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), "Publish feedback with color %s", feedback->current_color.c_str());

        if(last_msg->data == goal->color)
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

  void handle_subscription(const std_msgs::msg::String::SharedPtr msg)
  {
    last_msg = msg;
  }
};  // class GetIntakeColorServer


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GetIntakeColorServer>();

  rclcpp::spin(node);

  return 0;
}
