
#include "robotname_autonomy/plugins/action/set_speed.hpp" // Change This
#include "behaviortree_ros2/plugins.hpp"

BT::PortsList SetSpeed::providedPorts()
{
    return { InputPort<double>("x"), InputPort<double>("y"), InputPort<double>("theta"), InputPort<int>("timeout") };
}

bool SetSpeed::setGoal(RosActionNode::Goal &goal)
{

    auto speed_x = getInput<double>("x");
    auto speed_y = getInput<double>("y");
    auto theta = getInput<double>("theta");
    auto timeout = getInput<int>("timeout");
    
    goal.speed_x = speed_x.value();
    goal.speed_y = speed_y.value();
    goal.theta = theta.value();
    goal.timeout = timeout.value();

  return true;

}

NodeStatus SetSpeed::onResultReceived(const RosActionNode::WrappedResult &wr)
{

  if(wr.code == rclcpp_action::ResultCode::SUCCEEDED)
  {
    RCLCPP_INFO( node_->get_logger(), "[SetSpeed] Goal Reached");
    return NodeStatus::SUCCESS;
  }
  
    RCLCPP_INFO( node_->get_logger(), "[SetSpeed] Goal Failed");
    return NodeStatus::FAILURE;
}

NodeStatus SetSpeed::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
  
  RCLCPP_INFO( node_->get_logger(), "[SetSpeed] current_readings: %s", feedback->current_readings.c_str());
  
  return NodeStatus::RUNNING;
}

NodeStatus SetSpeed::onFailure(ActionNodeErrorCode error)
{
  RCLCPP_ERROR( node_->get_logger(), "[SetSpeed] %s: onFailure with error: %s", name().c_str(), toStr(error) );
  return NodeStatus::FAILURE;
}

void SetSpeed::onHalt()
{
  RCLCPP_INFO( node_->get_logger(), "[SetSpeed] %s: onHalt", name().c_str() );
}
