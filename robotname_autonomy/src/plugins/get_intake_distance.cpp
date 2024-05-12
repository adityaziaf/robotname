#include <std_msgs/msg/string.hpp>
#include "robotname_autonomy/plugins/get_intake_distance.hpp"

PortsList GetIntakeDistance::providedPorts()
{
    return { InputPort<std::string>("condition"),InputPort<float>("treshold"),OutputPort<std::string>("distance") };
}


bool GetIntakeDistance::setRequest(robotname_msgs::srv::GetIntakeDistance::Request::SharedPtr& request)
{
  (void)request;
    // use input ports to set A and B
    // must return true if we are ready to send the request
    return true;
}

NodeStatus GetIntakeDistance::onResponseReceived(const robotname_msgs::srv::GetIntakeDistance::Response::SharedPtr& response) 
  {
    
    if(response->status)
    {
      RCLCPP_INFO(node_->get_logger(), "[%s] Intake Distance Grab", name().c_str());
      
      float th;
      std::string condit;
      getInput("condition",condit);      
      getInput("treshold", th);
      setOutput("distance",response->distance);
      
      if(condit == "<")
      {
        if(response->distance < th)
        {
          return NodeStatus:: SUCCESS;
        }
        return NodeStatus:: FAILURE;
      }
      else if(condit == ">")
      {
        if(response->distance > th)
        {
          return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
      }
      
      //RCLCPP_INFO(node_->get_logger(), "Sum: %ld", response->sum);
    }
    RCLCPP_INFO(node_->get_logger(), "[%s] Failed retrieve Object Distance", name().c_str());
    return NodeStatus::FAILURE;
  }

NodeStatus GetIntakeDistance::onFailure(ServiceNodeErrorCode error) 
  {
    RCLCPP_ERROR(node_->get_logger(), "[%s] Error: %d", name().c_str(), error);
    return NodeStatus::FAILURE;
  }


