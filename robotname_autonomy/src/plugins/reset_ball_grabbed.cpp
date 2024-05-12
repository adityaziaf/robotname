#include "behaviortree_ros2/bt_service_node.hpp"
#include "robotname_autonomy/plugins/reset_ball_grabbed.hpp"

BT::PortsList ResetBallGrabbed::providedPorts()
{
    return {};
}

bool ResetBallGrabbed::setRequest(std_srvs::srv::SetBool::Request::SharedPtr& request)
{
    (void)request;
    // must return true if we are ready to send the request
    return true;
}

NodeStatus ResetBallGrabbed::onResponseReceived(const std_srvs::srv::SetBool::Response::SharedPtr& response) 
  {
    if(response->success)
    {
      RCLCPP_INFO(node_->get_logger(), "[%s] send reset ballgrabbed succesful", name().c_str());
      //RCLCPP_INFO(node_->get_logger(), "Sum: %ld", response->sum);
      return NodeStatus::SUCCESS;
    }
    RCLCPP_INFO(node_->get_logger(), "[%s] Failed send reset ballgrabbed", name().c_str());
    return NodeStatus::FAILURE; 
  }

NodeStatus ResetBallGrabbed::onFailure(ServiceNodeErrorCode error) 
  {
    RCLCPP_ERROR(node_->get_logger(), "[%s] Error: %d", name().c_str(), error);
    return NodeStatus::FAILURE;
  }






