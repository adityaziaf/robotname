#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include <std_msgs/msg/string.hpp>
#include "robotname_autonomy/plugins/get_intake_color.hpp"

PortsList ReceiveString::providedPorts()
{
    return { OutputPort<std::string>("color") };
}

NodeStatus ReceiveString::onTick(const std::shared_ptr<std_msgs::msg::String>& last_msg)
{
  if(last_msg) // empty if no new message received, since the last tick
  {
    std::string detectedcolor = last_msg->data;

    RCLCPP_INFO(logger(), "[%s] new message: %s", name().c_str(), detectedcolor.c_str());

    if(!first_msg)
    {
      first_msg = true;
      
    }
    else{
      if(prev_detectedcolor == detectedcolor)
      {
        return NodeStatus::FAILURE;
      }
    }
    setOutput("color", detectedcolor);
    prev_detectedcolor = detectedcolor;
  }
  else
  {
    return NodeStatus::FAILURE;
  }
  return NodeStatus::SUCCESS;
}


