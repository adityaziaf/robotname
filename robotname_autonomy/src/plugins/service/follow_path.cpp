#include "behaviortree_ros2/bt_service_node.hpp"
#include "robotname_autonomy/plugins/service/follow_path.hpp"
#include "robotname_autonomy/utils.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
#include "geometry_msgs/msg/point_stamped.hpp"

BT::PortsList FollowPose::providedPorts()
{
    return {InputPort<std::string>("frame_id"), InputPort<Pose>("goal")};
}

bool FollowPose::setRequest(robotname_msgs::srv::FollowPose::Request::SharedPtr& request)
{
    // use input ports to set A and B
    Pose targetpose;
    auto frame = getInput<std::string>("frame_id");
    getInput("goal", targetpose);
    
    geometry_msgs::msg::PoseStamped msg;
    
    msg.header.frame_id = frame.value();
    msg.header.stamp = node_->get_clock()->now();
    msg.pose.position.x = targetpose.x;
    msg.pose.position.y = targetpose.y;
    tf2::Quaternion q;
    q.setRPY(0, 0, targetpose.theta);
    q.normalize();
    msg.pose.orientation = tf2::toMsg(q);

    request->set__goal(msg);

    // must return true if we are ready to send the request
    return true;
}

NodeStatus FollowPose::onResponseReceived(const robotname_msgs::srv::FollowPose::Response::SharedPtr& response) 
  {
    if(response->status)
    {
      RCLCPP_INFO(node_->get_logger(), "[%s] follow path succesful", name().c_str());
      //RCLCPP_INFO(node_->get_logger(), "Sum: %ld", response->sum);
      return NodeStatus::SUCCESS;
    }
    RCLCPP_INFO(node_->get_logger(), "[%s] Failed follow path", name().c_str());
    return NodeStatus::FAILURE; 
  }

NodeStatus FollowPose::onFailure(ServiceNodeErrorCode error) 
  {
    RCLCPP_ERROR(node_->get_logger(), "[%s] Error: %d", name().c_str(), error);
    return NodeStatus::FAILURE;
  }








