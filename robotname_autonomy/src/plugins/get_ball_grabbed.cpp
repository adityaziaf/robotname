#include "robotname_autonomy/plugins/get_ball_grabbed.hpp"
#include "behaviortree_ros2/plugins.hpp"


BT::PortsList GetBallGrabbed::providedPorts()
{
    return { InputPort<std::string>("color_input"), OutputPort<std::string>("color_output")};
}
bool GetBallGrabbed::setGoal(RosActionNode::Goal &goal)
{
  auto data = getInput<std::string>("color_input");
  goal.color = data.value();

  return true;
}

NodeStatus GetBallGrabbed::onResultReceived(const RosActionNode::WrappedResult &wr)
{
//   RCLCPP_INFO( node_->get_logger(), "%s: onResultReceived. Done = %s", name().c_str(), 
//                wr.result->result );

  if(wr.code == rclcpp_action::ResultCode::SUCCEEDED)
  {
    RCLCPP_INFO( node_->get_logger(), "[GetBallGrabbed] Goal Reached");
    setOutput("color_output",wr.result->color);
    return NodeStatus::SUCCESS;
  }
  
    RCLCPP_INFO( node_->get_logger(), "[GetBallGrabbed] Goal Failed");
    return NodeStatus::FAILURE;
}

NodeStatus GetBallGrabbed::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
  
  //RCLCPP_INFO( node_->get_logger(), "[GetBallGrabbed] current color: %s", feedback->current_reading.c_str());
  
  return NodeStatus::RUNNING;
}

NodeStatus GetBallGrabbed::onFailure(ActionNodeErrorCode error)
{
  RCLCPP_ERROR( node_->get_logger(), "[GetBallGrabbed] %s: onFailure with error: %s", name().c_str(), toStr(error) );
  return NodeStatus::FAILURE;
}

void GetBallGrabbed::onHalt()
{
  RCLCPP_INFO( node_->get_logger(), "[GetBallGrabbed] %s: onHalt", name().c_str() );
}

// Plugin registration.
// The class GetBallGrabbed will self register with name  "Sleep".
// CreateRosNodePlugin(GetBallGrabbed, "GetBallGrabbed");
