
#include "robotname_autonomy/plugins/service/amcl_update.hpp"
#include "robotname_autonomy/utils.hpp"

BT::PortsList AMCLUpdate::providedPorts()
{
    return {};
}

bool AMCLUpdate::setRequest(std_srvs::srv::Empty::Request::SharedPtr& request)
{
    // must return true if we are ready to send the request
    return true;
}

NodeStatus AMCLUpdate::onResponseReceived(const std_srvs::srv::Empty::Response::SharedPtr& response) 
  {
    // if(response->status)
    // {
      
      RCLCPP_INFO(node_->get_logger(), "[%s] AMCL No Motion Update Sucsesfull", name().c_str());
      return NodeStatus::SUCCESS;
    }
    //RCLCPP_INFO(node_->get_logger(), "[%s] AMCL No Motion Update Failed", name().c_str());
  //}

NodeStatus AMCLUpdate::onFailure(ServiceNodeErrorCode error) 
  {
    RCLCPP_ERROR(node_->get_logger(), "[%s] Error: %d", name().c_str(), error);
    return NodeStatus::FAILURE;
  }




