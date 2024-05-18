
#include "robotname_autonomy/plugins/get_current_pose.hpp"
#include "robotname_autonomy/utils.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

BT::PortsList GetCurrentPose::providedPorts()
{
    return {InputPort<std::string>("parent_frame_id"),
    InputPort<std::string>("child_frame_id"), 
    OutputPort<Pose>("pose") };
}

bool GetCurrentPose::setRequest(robotname_msgs::srv::GetCurrentPose::Request::SharedPtr& request)
{
    // use input ports to set A and B
    getInput("parent_frame_id", request->parent_frame_id);
    getInput("child_frame_id", request->child_frame_id);
    // must return true if we are ready to send the request
    return true;
}

NodeStatus GetCurrentPose::onResponseReceived(const robotname_msgs::srv::GetCurrentPose::Response::SharedPtr& response) 
  {
    if(response->status)
    {
      
      Pose robotpose;
      robotpose.x = response->pose.pose.position.x;
      robotpose.y = response->pose.pose.position.y;
      robotpose.theta = tf2::getYaw(response->pose.pose.orientation);

      
      RCLCPP_INFO(node_->get_logger(), "[%s] Get Current Pose Succesfull", name().c_str());
      setOutput("pose",robotpose);
      //RCLCPP_INFO(node_->get_logger(), "Sum: %ld", response->sum);
      return NodeStatus::SUCCESS;
    }
    RCLCPP_INFO(node_->get_logger(), "[%s] Failed Tracked Current Pose", name().c_str());
    return NodeStatus::FAILURE; 
  }

NodeStatus GetCurrentPose::onFailure(ServiceNodeErrorCode error) 
  {
    RCLCPP_ERROR(node_->get_logger(), "[%s] Error: %d", name().c_str(), error);
    return NodeStatus::FAILURE;
  }




