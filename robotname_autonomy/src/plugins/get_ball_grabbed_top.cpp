#include "robotname_autonomy/plugins/get_ball_grabbed_top.hpp"
#include "behaviortree_ros2/plugins.hpp"


BT::PortsList GetBallGrabbedTop::providedPorts()
{
    return { InputPort<std::string>("color")};
}
bool GetBallGrabbedTop::setGoal(RosActionNode::Goal &goal)
{
  auto data = getInput<std::string>("color");
  goal.color = data.value();

  return true;
}

NodeStatus GetBallGrabbedTop::onResultReceived(const RosActionNode::WrappedResult &wr)
{
//   RCLCPP_INFO( node_->get_logger(), "%s: onResultReceived. Done = %s", name().c_str(), 
//                wr.result->result );

  if(wr.code == rclcpp_action::ResultCode::SUCCEEDED)
  {
    RCLCPP_INFO( node_->get_logger(), "[GetBallGrabbedTop] Goal Reached");
    return NodeStatus::SUCCESS;
  }
  
    RCLCPP_INFO( node_->get_logger(), "[GetBallGrabbedTop] Goal Failed");
    return NodeStatus::FAILURE;
}

NodeStatus GetBallGrabbedTop::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
  
  //RCLCPP_INFO( node_->get_logger(), "[GetBallGrabbedTop] current color: %s", feedback->current_reading.c_str());
  
  return NodeStatus::RUNNING;
}

NodeStatus GetBallGrabbedTop::onFailure(ActionNodeErrorCode error)
{
  RCLCPP_ERROR( node_->get_logger(), "[GetBallGrabbedTop] %s: onFailure with error: %s", name().c_str(), toStr(error) );
  return NodeStatus::FAILURE;
}

void GetBallGrabbedTop::onHalt()
{
  RCLCPP_INFO( node_->get_logger(), "[GetBallGrabbedTop] %s: onHalt", name().c_str() );
}

// Plugin registration.
// The class GetBallGrabbedTop will self register with name  "Sleep".
// CreateRosNodePlugin(GetBallGrabbedTop, "GetBallGrabbedTop");
