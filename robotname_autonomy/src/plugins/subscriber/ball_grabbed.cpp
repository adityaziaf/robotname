#include "robotname_autonomy/plugins/subscriber/ball_grabbed.hpp"

using namespace BT;

BT::PortsList BallGrabbed::providedPorts()
{
    return { InputPort<std::string>("color")};
}

NodeStatus BallGrabbed::onTick(const std::shared_ptr<std_msgs::msg::String>& last_msg)
{
  if(last_msg) // empty if no new message received, since the last tick
  {
      std::string ballcolor;

      getInput("color", ballcolor);

      if(last_msg->data == ballcolor)
      {
          return NodeStatus::SUCCESS;
      }
      
  }
      //RCLCPP_INFO(logger(), "[%s] new message: %s", name().c_str(), last_msg->data.c_str());      
  return NodeStatus::FAILURE;
}             
