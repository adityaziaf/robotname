#include "robotname_autonomy/plugins/intake_color.hpp"

using namespace BT;

BT::PortsList IntakeColor::providedPorts()
{
    return { OutputPort<std::string>("color")};
}

NodeStatus IntakeColor::onTick(const std::shared_ptr<std_msgs::msg::String>& last_msg)
{
  if(last_msg) // empty if no new message received, since the last tick
  {
      setOutput("color", last_msg->data);
      return NodeStatus::SUCCESS;
  }
      //RCLCPP_INFO(logger(), "[%s] new message: %s", name().c_str(), last_msg->data.c_str());      
  return NodeStatus::FAILURE;
}             
