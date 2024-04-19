#include "robotname_autonomy/plugins/nav_through_poses.hpp"
#include "behaviortree_ros2/plugins.hpp"
#include "robotname_autonomy/utils.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

BT::PortsList NavThroughPoses::providedPorts()
{
    return { InputPort<std::string>("frame_id"), InputPort<Poses>("goals") };
}

bool NavThroughPoses::setGoal(RosActionNode::Goal &goal)
{
  auto frame = getInput<std::string>("frame_id");
  auto res = getInput<Poses>("goals");
      if( !res )
      {
        throw BT::RuntimeError("error reading port [target]:", res.error());
      }
    Poses target_poses = res.value();

    for(auto &pose: target_poses)
    {
        geometry_msgs::msg::PoseStamped goal_pose;

        goal_pose.header.frame_id = frame.value();
        goal_pose.pose.position.x = pose.x;
        goal_pose.pose.position.y = pose.y;
        tf2::Quaternion q;
        q.setRPY(0, 0, pose.theta);
        q.normalize();
        goal_pose.pose.orientation = tf2::toMsg(q);

        goal.poses.push_back(goal_pose);
    }

  return true;
}

NodeStatus NavThroughPoses::onResultReceived(const RosActionNode::WrappedResult &wr)
{
//   RCLCPP_INFO( node_->get_logger(), "%s: onResultReceived. Done = %s", name().c_str(), 
//                wr.result->result );

  if(wr.code == rclcpp_action::ResultCode::SUCCEEDED)
  {
    RCLCPP_INFO( node_->get_logger(), "[NavThroughPoses] Goal Reached");
    return NodeStatus::SUCCESS;
  }
  
    RCLCPP_INFO( node_->get_logger(), "[NavThroughPoses] Goal Failed");
    return NodeStatus::FAILURE;
}

NodeStatus NavThroughPoses::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
  
  RCLCPP_INFO( node_->get_logger(), "[NavThroughPoses] remaining distance: %f", feedback->distance_remaining);
  
  return NodeStatus::RUNNING;
}

NodeStatus NavThroughPoses::onFailure(ActionNodeErrorCode error)
{
  RCLCPP_ERROR( node_->get_logger(), "[NavThroughPoses] %s: onFailure with error: %s", name().c_str(), toStr(error) );
  return NodeStatus::FAILURE;
}

void NavThroughPoses::onHalt()
{
  RCLCPP_INFO( node_->get_logger(), "[NavThroughPoses] %s: onHalt", name().c_str() );
}

// Plugin registration.
// The class NavThroughPoses will self register with name  "Sleep".
// CreateRosNodePlugin(NavThroughPoses, "NavThroughPoses");
