#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robotname_msgs/action/get_ball_grabbed.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"

#include "behaviortree_ros2/bt_action_node.hpp"

class GetBallGrabbedServer : public rclcpp::Node
{
public:
  using GetBallGrabbed = robotname_msgs::action::GetBallGrabbed;
  using GoalHandleGetBallGrabbed = rclcpp_action::ServerGoalHandle<GetBallGrabbed>;

  explicit GetBallGrabbedServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("GetBallGrabbed_action_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<GetBallGrabbed>(
      this,
      "get_ball_grabbed",
      std::bind(&GetBallGrabbedServer::handle_goal, this, _1, _2),
      std::bind(&GetBallGrabbedServer::handle_cancel, this, _1),
      std::bind(&GetBallGrabbedServer::handle_accepted, this, _1));
        
      _intake_prox_sub = this->create_subscription<std_msgs::msg::UInt8MultiArray>("/proximity_array", 1, std::bind(&GetBallGrabbedServer::handle_prox_subscription, this,_1));
      _intake_color_sub = this->create_subscription<std_msgs::msg::String>("/intake/detectedcolor", 1, std::bind(&GetBallGrabbedServer::handle_color_subscription, this,_1));
      _ballgrabbed_pub = this->create_publisher<std_msgs::msg::Bool>("/ballgrabbed",10);
    // 
  }

private:
  rclcpp_action::Server<GetBallGrabbed>::SharedPtr action_server_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr _intake_prox_sub;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _intake_color_sub;
  std_msgs::msg::String::SharedPtr last_color_msg;
  std_msgs::msg::UInt8MultiArray::SharedPtr last_prox_msg;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _ballgrabbed_pub;


  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const GetBallGrabbed::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with GetBallGrabbed %s", goal->color.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleGetBallGrabbed> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleGetBallGrabbed> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&GetBallGrabbedServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleGetBallGrabbed> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(100);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<GetBallGrabbed::Feedback>();
    auto result = std::make_shared<GetBallGrabbed::Result>();

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

      std_msgs::msg::Bool grabbedstate;
      grabbedstate.data = false;

      if(last_color_msg && last_prox_msg)
      {
        // feedback->current_reading = last_msg->data;
        // goal_handle->publish_feedback(feedback);
        // RCLCPP_INFO(this->get_logger(), "Publish feedback :%s", feedback->current_reading.c_str());

        // if(goal->pattern == last_msg->data)
        // {
        //   state = false;
        // }
        
        if(last_color_msg->data == goal->color)
        {
            if(last_prox_msg->data.front() == 1)
            {
                state = false;
                
                grabbedstate.data = true;

                _ballgrabbed_pub->publish(grabbedstate);
            }
            
        }
       
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "Empty Topic Value");
      }
      _ballgrabbed_pub->publish(grabbedstate);
      
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

  void handle_color_subscription(const std_msgs::msg::String::SharedPtr msg)
  {
    last_color_msg = msg;
  }

  void handle_prox_subscription(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
  {
    last_prox_msg = msg;
  }
};  // class GetBallGrabbedServer


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GetBallGrabbedServer>();

  rclcpp::spin(node);

  return 0;
}
