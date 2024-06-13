
#include "robotname_autonomy/plugins/service/get_best_silo.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "robotname_msgs/msg/detection_array.hpp"
#include "tf2/utils.h"

#include "robotname_autonomy/utils.hpp"

BT::PortsList GetBestSilo::providedPorts()
{
    return {OutputPort<int32_t>("silo"),
    InputPort<std::string>("team_color"), InputPort<int32_t>("mode")};
}

bool GetBestSilo::setRequest(robotname_msgs::srv::GetBestSilo::Request::SharedPtr& request)
{
    // use input ports to set A and B
    getInput("team_color", request->team_color);
    getInput("mode", request->mode);
    // must return true if we are ready to send the request
    return true;
}

NodeStatus GetBestSilo::onResponseReceived(const robotname_msgs::srv::GetBestSilo::Response::SharedPtr& response) 
  {
    if(response->status)
    {
      
      RCLCPP_INFO(node_->get_logger(), "[%s] Best Silo found %d", name().c_str(), response->silo);
      setOutput("silo",response->silo);
      //RCLCPP_INFO(node_->get_logger(), "Sum: %ld", response->sum);
      return NodeStatus::SUCCESS;
    }
    RCLCPP_INFO(node_->get_logger(), "[%s] Failed to get best Silo", name().c_str());
    return NodeStatus::FAILURE; 
  }

NodeStatus GetBestSilo::onFailure(ServiceNodeErrorCode error) 
  {
    RCLCPP_ERROR(node_->get_logger(), "[%s] Error: %d", name().c_str(), error);
    return NodeStatus::FAILURE;
  }




