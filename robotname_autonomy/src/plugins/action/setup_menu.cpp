#include "robotname_autonomy/plugins/action/setup_menu.hpp"
#include "behaviortree_ros2/plugins.hpp"


BT::PortsList SetupMenu::providedPorts()
{
    return { OutputPort<int>("silo2"), OutputPort<int>("silo3"),  OutputPort<int>("silo4") };
}

bool SetupMenu::setGoal(RosActionNode::Goal &goal)
{
  return true;
}

NodeStatus SetupMenu::onResultReceived(const RosActionNode::WrappedResult &wr)
{
//   RCLCPP_INFO( node_->get_logger(), "%s: onResultReceived. Done = %s", name().c_str(), 
//                wr.result->result );

  if(wr.code == rclcpp_action::ResultCode::SUCCEEDED)
  {
    RCLCPP_INFO( node_->get_logger(), "[SetupMenu] Goal Reached");
    setOutput("silo2", wr.result->silo2);
    setOutput("silo3", wr.result->silo3);
    setOutput("silo4", wr.result->silo4);
    return NodeStatus::SUCCESS;
  }
  
    RCLCPP_INFO( node_->get_logger(), "[SetupMenu] Goal Failed");
    return NodeStatus::FAILURE;
}

NodeStatus SetupMenu::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
  
  //RCLCPP_INFO( node_->get_logger(), "[GetBallGrabbedTop] current color: %s", feedback->current_reading.c_str());
  
  return NodeStatus::RUNNING;
}

NodeStatus SetupMenu::onFailure(ActionNodeErrorCode error)
{
  RCLCPP_ERROR( node_->get_logger(), "[SetupMenu] %s: onFailure with error: %s", name().c_str(), toStr(error) );
  return NodeStatus::FAILURE;
}

void SetupMenu::onHalt()
{
  RCLCPP_INFO( node_->get_logger(), "[SetupMenu] %s: onHalt", name().c_str() );
}

// Plugin registration.
// The class GetBallGrabbedTop will self register with name  "Sleep".
// CreateRosNodePlugin(GetBallGrabbedTop, "GetBallGrabbedTop");
