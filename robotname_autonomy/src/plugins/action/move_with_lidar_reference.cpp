
#include "robotname_autonomy/plugins/action/move_with_lidar_reference.hpp"
#include "behaviortree_ros2/plugins.hpp"

BT::PortsList MoveWithLidarReference::providedPorts()
{
    return { InputPort<std::string>("parent_frame"), InputPort<std::string>("child_frame"), 
             InputPort<double>("x"), InputPort<double>("x") };
}

bool MoveWithLidarReference::setGoal(RosActionNode::Goal &goal)
{
  auto parent_frame = getInput<std::string>("parent_frame");
  auto child_frame = getInput<std::string>("child_frame");
  auto x = getInput<double>("x");
  auto y = getInput<double>("y");
  goal.x = x.value();
  goal.y = y.value();

  return true;
}

NodeStatus MoveWithLidarReference::onResultReceived(const RosActionNode::WrappedResult &wr)
{
//   RCLCPP_INFO( node_->get_logger(), "%s: onResultReceived. Done = %s", name().c_str(), 
//                wr.result->result );

  if(wr.code == rclcpp_action::ResultCode::SUCCEEDED)
  {
    RCLCPP_INFO( node_->get_logger(), "[MoveWithLidarReference] Goal Reached");
    return NodeStatus::SUCCESS;
  }
  
    RCLCPP_INFO( node_->get_logger(), "[MoveWithLidarReference] Goal Failed");
    return NodeStatus::FAILURE;
}

NodeStatus MoveWithLidarReference::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
  
  RCLCPP_INFO( node_->get_logger(), "[MoveWithLidarReference] remaining distance: %f", feedback->remaining_distance);
  
  return NodeStatus::RUNNING;
}

NodeStatus MoveWithLidarReference::onFailure(ActionNodeErrorCode error)
{
  RCLCPP_ERROR( node_->get_logger(), "[MoveWithLidarReference] %s: onFailure with error: %s", name().c_str(), toStr(error) );
  return NodeStatus::FAILURE;
}

void MoveWithLidarReference::onHalt()
{
  RCLCPP_INFO( node_->get_logger(), "[MoveWithLidarReference] %s: onHalt", name().c_str() );
}

// Plugin registration.
// The class MoveWithLidarReference will self register with name  "Sleep".
// CreateRosNodePlugin(MoveWithLidarReference, "MoveWithLidarReference");
