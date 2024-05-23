#include "robotname_autonomy/plugins/subscriber/id_found.hpp"

using namespace BT;

NodeStatus IdFound::onTick(const std::shared_ptr<std_msgs::msg::Bool>& last_msg)
{
  if(last_msg) // empty if no new message received, since the last tick
  {

      if(last_msg->data == true)
      {
          return NodeStatus::SUCCESS;
      }
      
  }
      //RCLCPP_INFO(logger(), "[%s] new message: %s", name().c_str(), last_msg->data.c_str());      
  return NodeStatus::FAILURE;
}             
