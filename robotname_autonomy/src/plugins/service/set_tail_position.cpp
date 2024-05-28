#include "behaviortree_ros2/bt_service_node.hpp"
#include "robotname_autonomy/plugins/service/set_tail_position.hpp"
#include "robotname_autonomy/utils.hpp"
#include "robotname_msgs/srv/set_tail_position.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

BT::PortsList SetTailPosition::providedPorts()
{
    return {InputPort<float>("angle")};
}

bool SetTailPosition::setRequest(robotname_msgs::srv::SetTailPosition::Request::SharedPtr& request)
{
    // use input ports to set A and B
    
    getInput("angle", request->angle);

    // must return true if we are ready to send the request
    return true;
}

NodeStatus SetTailPosition::onResponseReceived(const robotname_msgs::srv::SetTailPosition::Response::SharedPtr& response) 
  {
    if(response->status)
    {
      RCLCPP_INFO(node_->get_logger(), "[%s] send set tail position succesful", name().c_str());
      //RCLCPP_INFO(node_->get_logger(), "Sum: %ld", response->sum);
      return NodeStatus::SUCCESS;
    }
    RCLCPP_INFO(node_->get_logger(), "[%s] Failed send set tail position", name().c_str());
    return NodeStatus::FAILURE; 
  }

NodeStatus SetTailPosition::onFailure(ServiceNodeErrorCode error) 
  {
    RCLCPP_ERROR(node_->get_logger(), "[%s] Error: %d", name().c_str(), error);
    return NodeStatus::FAILURE;
  }






