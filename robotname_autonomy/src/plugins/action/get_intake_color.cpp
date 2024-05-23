#include "robotname_autonomy/plugins/action/get_intake_color.hpp"
#include "behaviortree_ros2/plugins.hpp"


BT::PortsList GetIntakeColor::providedPorts()
{
    return { InputPort<std::string>("color")};
}
bool GetIntakeColor::setGoal(RosActionNode::Goal &goal)
{
  auto data = getInput<std::string>("color");
  goal.color = data.value();

  return true;
}

NodeStatus GetIntakeColor::onResultReceived(const RosActionNode::WrappedResult &wr)
{
//   RCLCPP_INFO( node_->get_logger(), "%s: onResultReceived. Done = %s", name().c_str(), 
//                wr.result->result );

  if(wr.code == rclcpp_action::ResultCode::SUCCEEDED)
  {
    RCLCPP_INFO( node_->get_logger(), "[GetIntakeColor] Goal Reached");
    return NodeStatus::SUCCESS;
  }
  
    RCLCPP_INFO( node_->get_logger(), "[GetIntakeColor] Goal Failed");
    return NodeStatus::FAILURE;
}

NodeStatus GetIntakeColor::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
  
  RCLCPP_INFO( node_->get_logger(), "[GetIntakeColor] current color: %s", feedback->current_color.c_str());
  
  return NodeStatus::RUNNING;
}

NodeStatus GetIntakeColor::onFailure(ActionNodeErrorCode error)
{
  RCLCPP_ERROR( node_->get_logger(), "[GetIntakeColor] %s: onFailure with error: %s", name().c_str(), toStr(error) );
  return NodeStatus::FAILURE;
}

void GetIntakeColor::onHalt()
{
  RCLCPP_INFO( node_->get_logger(), "[GetIntakeColor] %s: onHalt", name().c_str() );
}

// Plugin registration.
// The class GetIntakeColor will self register with name  "Sleep".
// CreateRosNodePlugin(GetIntakeColor, "GetIntakeColor");
