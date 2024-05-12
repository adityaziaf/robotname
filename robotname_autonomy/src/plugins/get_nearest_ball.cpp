
#include "robotname_autonomy/plugins/get_nearest_ball.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "robotname_msgs/msg/detection_array.hpp"
#include "tf2/utils.h"

#include "robotname_autonomy/utils.hpp"

BT::PortsList GetNearestBall::providedPorts()
{
    return {OutputPort<int32_t>("id"),
    InputPort<std::string>("color")};
}

bool GetNearestBall::setRequest(robotname_msgs::srv::FindNearestBall::Request::SharedPtr& request)
{
    // use input ports to set A and B
    getInput("color", request->color);
    // must return true if we are ready to send the request
    return true;
}

NodeStatus GetNearestBall::onResponseReceived(const robotname_msgs::srv::FindNearestBall::Response::SharedPtr& response) 
  {
    if(response->status)
    {
      
      RCLCPP_INFO(node_->get_logger(), "[%s] Ball Found with ID %d", name().c_str(), response->id);
      setOutput("id",response->id);
      //RCLCPP_INFO(node_->get_logger(), "Sum: %ld", response->sum);
      return NodeStatus::SUCCESS;
    }
    RCLCPP_INFO(node_->get_logger(), "[%s] Failed Get Ball", name().c_str());
    return NodeStatus::FAILURE; 
  }

NodeStatus GetNearestBall::onFailure(ServiceNodeErrorCode error) 
  {
    RCLCPP_ERROR(node_->get_logger(), "[%s] Error: %d", name().c_str(), error);
    return NodeStatus::FAILURE;
  }




