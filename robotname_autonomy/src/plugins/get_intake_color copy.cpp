#include <std_msgs/msg/string.hpp>
#include "robotname_autonomy/plugins/get_intake_color.hpp"

PortsList GetIntakeColor::providedPorts()
{
    return { OutputPort<std::string>("color") };
}


bool GetIntakeColor::setRequest(std_srvs::srv::Trigger::Request::SharedPtr& request)
{
  (void)request;
    // use input ports to set A and B
    // must return true if we are ready to send the request
    return true;
}

NodeStatus GetIntakeColor::onResponseReceived(const std_srvs::srv::Trigger::Response::SharedPtr& response) 
  {
    
    if(response->success)
    {
      RCLCPP_INFO(node_->get_logger(), "[%s] IntakeBall Found with color %s", name().c_str(), response->message.c_str());
      setOutput("color",response->message);
      //RCLCPP_INFO(node_->get_logger(), "Sum: %ld", response->sum);
      return NodeStatus::SUCCESS;
    }
    RCLCPP_INFO(node_->get_logger(), "[%s] Failed retrieve Intakeball", name().c_str());
    return NodeStatus::FAILURE;
  }

NodeStatus GetIntakeColor::onFailure(ServiceNodeErrorCode error) 
  {
    RCLCPP_ERROR(node_->get_logger(), "[%s] Error: %d", name().c_str(), error);
    return NodeStatus::FAILURE;
  }


