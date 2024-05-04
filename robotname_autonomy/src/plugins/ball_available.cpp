
#include "robotname_autonomy/plugins/ball_available.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "robotname_msgs/msg/detection_array.hpp"
#include "tf2/utils.h"

#include "robotname_autonomy/utils.hpp"

BT::PortsList BallAvailable::providedPorts()
{
    return {InputPort<int32_t>("id"),
    InputPort<std::string>("color"), 
    OutputPort<Pose>("position") };
}

bool BallAvailable::setRequest(robotname_msgs::srv::BallAvailable::Request::SharedPtr& request)
{
    // use input ports to set A and B
    getInput("color", request->color);
    getInput("id", request->id);
    // must return true if we are ready to send the request
    return true;
}

NodeStatus BallAvailable::onResponseReceived(const robotname_msgs::srv::BallAvailable::Response::SharedPtr& response) 
  {
    if(response->status)
    {
      Pose ballpos;
      ballpos.x = response->pose.pose.position.x;
      ballpos.y = response->pose.pose.position.y;
      ballpos.theta = tf2::getYaw(response->pose.pose.orientation);
      
      RCLCPP_INFO(node_->get_logger(), "[%s] Ball Tracked with position %lf:%lf:%lf", name().c_str(), ballpos.x, ballpos.y, ballpos.theta);
      setOutput("position",ballpos);
      //RCLCPP_INFO(node_->get_logger(), "Sum: %ld", response->sum);
      return NodeStatus::SUCCESS;
    }
    RCLCPP_INFO(node_->get_logger(), "[%s] Failed Tracked Ball", name().c_str());
    return NodeStatus::FAILURE; 
  }

NodeStatus BallAvailable::onFailure(ServiceNodeErrorCode error) 
  {
    RCLCPP_ERROR(node_->get_logger(), "[%s] Error: %d", name().c_str(), error);
    return NodeStatus::FAILURE;
  }




