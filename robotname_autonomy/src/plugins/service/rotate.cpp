
#include "robotname_autonomy/plugins/service/rotate.hpp"
#include "robotname_autonomy/utils.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

BT::PortsList Rotate::providedPorts()
{
    return {InputPort<Pose>("pose"),
    InputPort<std::string>("frame_id"), 
    InputPort<float>("angle"),
    OutputPort<Pose>("rotated_pose")};
}

bool Rotate::setRequest(robotname_msgs::srv::Rotate::Request::SharedPtr& request)
{
    // use input ports to set A and B
    Pose input;
    std::string frame;
    getInput("pose", input);
    getInput("frame_id", frame);

    request->pose.header.frame_id = frame;
    request->pose.pose.position.x = input.x;
    request->pose.pose.position.y = input.y;
    tf2::Quaternion q;
    q.setRPY(0, 0, input.theta);
    q.normalize();
    request->pose.pose.orientation = tf2::toMsg(q);
    
    getInput("angle", request->angle);
    // must return true if we are ready to send the request
    return true;
}

NodeStatus Rotate::onResponseReceived(const robotname_msgs::srv::Rotate::Response::SharedPtr& response) 
  {
    if(response->status)
    {
      Pose rotpos;
      rotpos.x = response->pose.pose.position.x;
      rotpos.y = response->pose.pose.position.y;
      rotpos.theta = tf2::getYaw(response->pose.pose.orientation);
      
      RCLCPP_INFO(node_->get_logger(), "[%s] Rotate Sucsesfull", name().c_str());
      setOutput("rotated_pose",rotpos);
      //RCLCPP_INFO(node_->get_logger(), "Sum: %ld", response->sum);
      return NodeStatus::SUCCESS;
    }
    RCLCPP_INFO(node_->get_logger(), "[%s] Failed Tracked Ball", name().c_str());
    return NodeStatus::FAILURE; 
  }

NodeStatus Rotate::onFailure(ServiceNodeErrorCode error) 
  {
    RCLCPP_ERROR(node_->get_logger(), "[%s] Error: %d", name().c_str(), error);
    return NodeStatus::FAILURE;
  }




