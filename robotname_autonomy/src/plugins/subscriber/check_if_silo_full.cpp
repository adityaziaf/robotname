#include "robotname_autonomy/plugins/subscriber/check_if_silo_full.hpp"

using namespace BT;

NodeStatus CheckIfSiloFull::onTick(const std::shared_ptr<std_msgs::msg::String>& last_msg)
{
  if(last_msg) // empty if no new message received, since the last tick
  {
      if(last_msg->data != "null")
      {
          return NodeStatus::SUCCESS;
      }
      
  }
      //RCLCPP_INFO(logger(), "[%s] new message: %s", name().c_str(), last_msg->data.c_str());      
  return NodeStatus::FAILURE;
}             
