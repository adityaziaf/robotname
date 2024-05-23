#include "robotname_autonomy/plugins/action/wait_button.hpp"
#include "behaviortree_ros2/plugins.hpp"


bool WaitButton::setGoal(RosActionNode::Goal &goal)
{
  return true;
}

NodeStatus WaitButton::onResultReceived(const RosActionNode::WrappedResult &wr)
{
//   RCLCPP_INFO( node_->get_logger(), "%s: onResultReceived. Done = %s", name().c_str(), 
//                wr.result->result );

  if(wr.code == rclcpp_action::ResultCode::SUCCEEDED)
  {
    RCLCPP_INFO( node_->get_logger(), "[WaitButton] Goal Reached");
    return NodeStatus::SUCCESS;
  }
  
    RCLCPP_INFO( node_->get_logger(), "[WaitButton] Goal Failed");
    return NodeStatus::FAILURE;
}

NodeStatus WaitButton::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
  
  //RCLCPP_INFO( node_->get_logger(), "[GetBallGrabbedTop] current color: %s", feedback->current_reading.c_str());
  
  return NodeStatus::RUNNING;
}

NodeStatus WaitButton::onFailure(ActionNodeErrorCode error)
{
  RCLCPP_ERROR( node_->get_logger(), "[WaitButton] %s: onFailure with error: %s", name().c_str(), toStr(error) );
  return NodeStatus::FAILURE;
}

void WaitButton::onHalt()
{
  RCLCPP_INFO( node_->get_logger(), "[WaitButton] %s: onHalt", name().c_str() );
}

// Plugin registration.
// The class GetBallGrabbedTop will self register with name  "Sleep".
// CreateRosNodePlugin(GetBallGrabbedTop, "GetBallGrabbedTop");
