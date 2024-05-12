#include "robotname_autonomy/plugins/follow_ball.hpp"
#include "behaviortree_ros2/plugins.hpp"


BT::PortsList Follow::providedPorts()
{
    return { InputPort<std::string>("color"), InputPort<int>("id")};
}
bool Follow::setGoal(RosActionNode::Goal &goal)
{
  auto data = getInput<std::string>("color");
  auto id = getInput<int>("id");

  goal.color = data.value();
  goal.id = id.value();

  return true;
}

NodeStatus Follow::onResultReceived(const RosActionNode::WrappedResult &wr)
{
//   RCLCPP_INFO( node_->get_logger(), "%s: onResultReceived. Done = %s", name().c_str(), 
//                wr.result->result );

  if(wr.result->status)
  {
    //RCLCPP_INFO( node_->get_logger(), "[Follow] Ball found with id:%d", wr.result->id);
    return NodeStatus::SUCCESS;
  }
  
    RCLCPP_INFO( node_->get_logger(), "[Follow] Ball not found");
    return NodeStatus::FAILURE;
}

NodeStatus Follow::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
  
  RCLCPP_INFO( node_->get_logger(), "[Follow] following ball");
  
  return NodeStatus::RUNNING;
}

NodeStatus Follow::onFailure(ActionNodeErrorCode error)
{
  RCLCPP_ERROR( node_->get_logger(), "[Follow] %s: onFailure with error: %s", name().c_str(), toStr(error) );
  return NodeStatus::FAILURE;
}

void Follow::onHalt()
{
  RCLCPP_INFO( node_->get_logger(), "[Follow] %s: onHalt", name().c_str() );
}

// Plugin registration.
// The class Follow will self register with name  "Sleep".
// CreateRosNodePlugin(Follow, "Follow");
