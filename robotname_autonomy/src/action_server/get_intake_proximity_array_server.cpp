#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robotname_msgs/action/get_intake_proximity_array.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "behaviortree_ros2/bt_action_node.hpp"

class GetIntakeProximityArrayServer : public rclcpp::Node
{
public:
  using GetIntakeProximityArray = robotname_msgs::action::GetIntakeProximityArray;
  using GoalHandleGetIntakeProximityArray = rclcpp_action::ServerGoalHandle<GetIntakeProximityArray>;

  explicit GetIntakeProximityArrayServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("GetIntakeProximityArray_action_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<GetIntakeProximityArray>(
      this,
      "get_intake_proximity_array",
      std::bind(&GetIntakeProximityArrayServer::handle_goal, this, _1, _2),
      std::bind(&GetIntakeProximityArrayServer::handle_cancel, this, _1),
      std::bind(&GetIntakeProximityArrayServer::handle_accepted, this, _1));

    _intake_sub = this->create_subscription<std_msgs::msg::UInt8MultiArray>("/proximity_array", 1, std::bind(&GetIntakeProximityArrayServer::handle_subscription, this,_1));
  }

private:
  rclcpp_action::Server<GetIntakeProximityArray>::SharedPtr action_server_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr _intake_sub;
  std_msgs::msg::UInt8MultiArray::SharedPtr last_msg;


  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const GetIntakeProximityArray::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with GetIntakeProximityArray %s", goal->pattern.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleGetIntakeProximityArray> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleGetIntakeProximityArray> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&GetIntakeProximityArrayServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleGetIntakeProximityArray> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(100);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<GetIntakeProximityArray::Feedback>();
    auto result = std::make_shared<GetIntakeProximityArray::Result>();

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
        // feedback->current_reading = last_msg->data;
        // goal_handle->publish_feedback(feedback);
        // RCLCPP_INFO(this->get_logger(), "Publish feedback :%s", feedback->current_reading.c_str());
        std::size_t counter_hit = 0;
        if(last_msg->data.size() == goal->pattern.size())
        {
          for(std::size_t i = 0; i < goal->pattern.size(); i++)
          {
            if(last_msg->data[i] == (goal->pattern[i] - '0'))
            {
              counter_hit++;
              //RCLCPP_INFO(this->get_logger(), "%ld" ,counter_hit);
            }
          }

          if(counter_hit == goal->pattern.size())
          {
            state = false;
            //RCLCPP_INFO(this->get_logger(), "Hit");
          }
          //RCLCPP_INFO(this->get_logger(), "Hit1");
        }
        else{
          //RCLCPP_INFO(this->get_logger(), "Container have different size");
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

  void handle_subscription(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
  {
    last_msg = msg;
  }
};  // class GetIntakeProximityArrayServer


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GetIntakeProximityArrayServer>();

  rclcpp::spin(node);

  return 0;
}
