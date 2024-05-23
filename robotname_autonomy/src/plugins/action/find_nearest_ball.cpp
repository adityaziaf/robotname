#include "robotname_autonomy/plugins/action/find_nearest_ball.hpp"
#include "behaviortree_ros2/plugins.hpp"


BT::PortsList FindNearestBall::providedPorts()
{
    return { InputPort<std::string>("color"), OutputPort<int>("id")};
}
bool FindNearestBall::setGoal(RosActionNode::Goal &goal)
{
  auto data = getInput<std::string>("color");
  goal.color = data.value();

  return true;
}

NodeStatus FindNearestBall::onResultReceived(const RosActionNode::WrappedResult &wr)
{
//   RCLCPP_INFO( node_->get_logger(), "%s: onResultReceived. Done = %s", name().c_str(), 
//                wr.result->result );

  if(wr.code == rclcpp_action::ResultCode::SUCCEEDED)
  {
    setOutput("id", wr.result->id);
    RCLCPP_INFO( node_->get_logger(), "[FindNearestBall] Ball found with id:%d", wr.result->id);
    return NodeStatus::SUCCESS;
  }
  
    RCLCPP_INFO( node_->get_logger(), "[FindNearestBall] Ball not found");
    return NodeStatus::FAILURE;
}

NodeStatus FindNearestBall::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
  
  //RCLCPP_INFO( node_->get_logger(), "[FindNearestBall] current color: %s", feedback->current_reading.c_str());
  
  return NodeStatus::RUNNING;
}

NodeStatus FindNearestBall::onFailure(ActionNodeErrorCode error)
{
  RCLCPP_ERROR( node_->get_logger(), "[FindNearestBall] %s: onFailure with error: %s", name().c_str(), toStr(error) );
  return NodeStatus::FAILURE;
}

void FindNearestBall::onHalt()
{
  RCLCPP_INFO( node_->get_logger(), "[FindNearestBall] %s: onHalt", name().c_str() );
}

// Plugin registration.
// The class FindNearestBall will self register with name  "Sleep".
// CreateRosNodePlugin(FindNearestBall, "FindNearestBall");
