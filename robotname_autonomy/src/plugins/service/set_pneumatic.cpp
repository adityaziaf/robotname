#include "behaviortree_ros2/bt_service_node.hpp"
#include "robotname_autonomy/plugins/service/set_pneumatic.hpp"
#include "robotname_autonomy/utils.hpp"
#include "robotname_msgs/srv/set_pneumatic.hpp"
#include "std_msgs/msg/bool.hpp"

BT::PortsList SetPneumatic::providedPorts()
{
    return { InputPort<bool>("pneumatic") };
}

bool SetPneumatic::setRequest(robotname_msgs::srv::SetPneumatic::Request::SharedPtr& request)
{
    // use input ports to set A and B
    getInput("pneumatic", request->pneumatic);

    // must return true if we are ready to send the request
    return true;

}

NodeStatus SetPneumatic::onResponseReceived(const robotname_msgs::srv::SetPneumatic::Response::SharedPtr& response) 
  {
    if(response->status)
    {
      RCLCPP_INFO(node_->get_logger(), "[%s] send pneumatic succesful", name().c_str());
      //RCLCPP_INFO(node_->get_logger(), "Sum: %ld", response->sum);
      return NodeStatus::SUCCESS;
    }
    RCLCPP_INFO(node_->get_logger(), "[%s] Failed send pneumatic", name().c_str());
    return NodeStatus::FAILURE; 
  }

NodeStatus SetPneumatic::onFailure(ServiceNodeErrorCode error) 
  {
    RCLCPP_ERROR(node_->get_logger(), "[%s] Error: %d", name().c_str(), error);
    return NodeStatus::FAILURE;
  }

