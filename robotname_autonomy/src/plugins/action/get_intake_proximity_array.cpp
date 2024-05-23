#include "robotname_autonomy/plugins/action/get_intake_proximity_array.hpp"
#include "behaviortree_ros2/plugins.hpp"
#include "std_msgs/msg/u_int8_multi_array.h"

BT::PortsList GetIntakeProximityArray::providedPorts()
{
    return { InputPort<std::string>("pattern")};
}
bool GetIntakeProximityArray::setGoal(RosActionNode::Goal &goal)
{
  auto data = getInput<std::string>("pattern");
  goal.pattern = data.value();

  return true;
}

NodeStatus GetIntakeProximityArray::onResultReceived(const RosActionNode::WrappedResult &wr)
{
//   RCLCPP_INFO( node_->get_logger(), "%s: onResultReceived. Done = %s", name().c_str(), 
//                wr.result->result );

  if(wr.code == rclcpp_action::ResultCode::SUCCEEDED)
  {
    RCLCPP_INFO( node_->get_logger(), "[GetIntakeProximityArray] Goal Reached");
    return NodeStatus::SUCCESS;
  }
  
    RCLCPP_INFO( node_->get_logger(), "[GetIntakeProximityArray] Goal Failed");
    return NodeStatus::FAILURE;
}

NodeStatus GetIntakeProximityArray::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
  
  RCLCPP_INFO( node_->get_logger(), "[GetIntakeProximityArray] current color: %s", feedback->current_reading.c_str());
  
  return NodeStatus::RUNNING;
}

NodeStatus GetIntakeProximityArray::onFailure(ActionNodeErrorCode error)
{
  RCLCPP_ERROR( node_->get_logger(), "[GetIntakeProximityArray] %s: onFailure with error: %s", name().c_str(), toStr(error) );
  return NodeStatus::FAILURE;
}

void GetIntakeProximityArray::onHalt()
{
  RCLCPP_INFO( node_->get_logger(), "[GetIntakeProximityArray] %s: onHalt", name().c_str() );
}

// Plugin registration.
// The class GetIntakeProximityArray will self register with name  "Sleep".
// CreateRosNodePlugin(GetIntakeProximityArray, "GetIntakeProximityArray");
