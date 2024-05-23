
#include "robotname_autonomy/plugins/action/rotate_speed.hpp"
#include "behaviortree_ros2/plugins.hpp"

BT::PortsList RotateSpeed::providedPorts()
{
    return {InputPort<double>("speed"), InputPort<double>("angle") };
}

bool RotateSpeed::setGoal(RosActionNode::Goal &goal)
{
  auto speed = getInput<double>("speed");
  auto angle = getInput<double>("angle");
  goal.speed = speed.value();
  goal.angle = angle.value();

  return true;
}

NodeStatus RotateSpeed::onResultReceived(const RosActionNode::WrappedResult &wr)
{
//   RCLCPP_INFO( node_->get_logger(), "%s: onResultReceived. Done = %s", name().c_str(), 
//                wr.result->result );

  if(wr.code == rclcpp_action::ResultCode::SUCCEEDED)
  {
    RCLCPP_INFO( node_->get_logger(), "[RotateSpeed] Goal Reached");
    return NodeStatus::SUCCESS;
  }
  
    RCLCPP_INFO( node_->get_logger(), "[RotateSpeed] Goal Failed");
    return NodeStatus::FAILURE;
}

NodeStatus RotateSpeed::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
  
  RCLCPP_INFO( node_->get_logger(), "[RotateSpeed] current color");
  
  return NodeStatus::RUNNING;
}

NodeStatus RotateSpeed::onFailure(ActionNodeErrorCode error)
{
  RCLCPP_ERROR( node_->get_logger(), "[RotateSpeed] %s: onFailure with error: %s", name().c_str(), toStr(error) );
  return NodeStatus::FAILURE;
}

void RotateSpeed::onHalt()
{
  RCLCPP_INFO( node_->get_logger(), "[RotateSpeed] %s: onHalt", name().c_str() );
}

// Plugin registration.
// The class RotateSpeed will self register with name  "Sleep".
// CreateRosNodePlugin(RotateSpeed, "RotateSpeed");
