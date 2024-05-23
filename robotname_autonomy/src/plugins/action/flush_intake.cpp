#include "robotname_autonomy/plugins/action/flush_intake.hpp"
#include "behaviortree_ros2/plugins.hpp"


BT::PortsList FlushIntake::providedPorts()
{
    return { InputPort<int32_t>("target_remaining_ball")};
}
bool FlushIntake::setGoal(RosActionNode::Goal &goal)
{
  auto data = getInput<int32_t>("target_remaining_ball");
  goal.target_remaining_ball = data.value();

  return true;
}

NodeStatus FlushIntake::onResultReceived(const RosActionNode::WrappedResult &wr)
{
//   RCLCPP_INFO( node_->get_logger(), "%s: onResultReceived. Done = %s", name().c_str(), 
//                wr.result->result );

  if(wr.code == rclcpp_action::ResultCode::SUCCEEDED)
  {
    RCLCPP_INFO( node_->get_logger(), "[FlushIntake] Goal Reached");
    return NodeStatus::SUCCESS;
  }
  
    RCLCPP_INFO( node_->get_logger(), "[FlushIntake] Goal Failed");
    return NodeStatus::FAILURE;
}

NodeStatus FlushIntake::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
  
  RCLCPP_INFO( node_->get_logger(), "[FlushIntake] current color: %d", feedback->remaining_ball);
  
  return NodeStatus::RUNNING;
}

NodeStatus FlushIntake::onFailure(ActionNodeErrorCode error)
{
  RCLCPP_ERROR( node_->get_logger(), "[FlushIntake] %s: onFailure with error: %s", name().c_str(), toStr(error) );
  return NodeStatus::FAILURE;
}

void FlushIntake::onHalt()
{
  RCLCPP_INFO( node_->get_logger(), "[FlushIntake] %s: onHalt", name().c_str() );
}

// Plugin registration.
// The class FlushIntake will self register with name  "Sleep".
// CreateRosNodePlugin(FlushIntake, "FlushIntake");
