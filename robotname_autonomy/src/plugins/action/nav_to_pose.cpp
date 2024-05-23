#include "robotname_autonomy/plugins/action/nav_to_pose.hpp"
#include "behaviortree_ros2/plugins.hpp"
#include "robotname_autonomy/utils.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

BT::PortsList NavToPose::providedPorts()
{
    return { InputPort<std::string>("frame_id"), InputPort<Pose>("goal") };
}
bool NavToPose::setGoal(RosActionNode::Goal &goal)
{
  auto frame = getInput<std::string>("frame_id");
  auto res = getInput<Pose>("goal");
      if( !res )
      {
        throw BT::RuntimeError("error reading port [target]:", res.error());
      }
      Pose target_pose = res.value();

    goal.pose.header.frame_id = frame.value();
    goal.pose.pose.position.x = target_pose.x;
    goal.pose.pose.position.y = target_pose.y;
    tf2::Quaternion q;
    q.setRPY(0, 0, target_pose.theta);
    q.normalize();
    goal.pose.pose.orientation = tf2::toMsg(q);

  return true;
}

NodeStatus NavToPose::onResultReceived(const RosActionNode::WrappedResult &wr)
{
//   RCLCPP_INFO( node_->get_logger(), "%s: onResultReceived. Done = %s", name().c_str(), 
//                wr.result->result );

  if(wr.code == rclcpp_action::ResultCode::SUCCEEDED)
  {
    RCLCPP_INFO( node_->get_logger(), "[NavToPose] Goal Reached");
    return NodeStatus::SUCCESS;
  }
  
    RCLCPP_INFO( node_->get_logger(), "[NavToPose] Goal Failed");
    return NodeStatus::FAILURE;
}

NodeStatus NavToPose::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
  
  RCLCPP_INFO( node_->get_logger(), "[NavToPose] remaining distance: %f", feedback->distance_remaining);
  
  return NodeStatus::RUNNING;
}

NodeStatus NavToPose::onFailure(ActionNodeErrorCode error)
{
  RCLCPP_ERROR( node_->get_logger(), "[NavToPose] %s: onFailure with error: %s", name().c_str(), toStr(error) );
  return NodeStatus::FAILURE;
}

void NavToPose::onHalt()
{
  RCLCPP_INFO( node_->get_logger(), "[NavToPose] %s: onHalt", name().c_str() );
}

// Plugin registration.
// The class NavToPose will self register with name  "Sleep".
// CreateRosNodePlugin(NavToPose, "NavToPose");
