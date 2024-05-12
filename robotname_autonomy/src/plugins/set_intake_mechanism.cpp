#include "behaviortree_ros2/bt_service_node.hpp"
#include "robotname_autonomy/plugins/set_intake_mechanism.hpp"
#include "robotname_autonomy/utils.hpp"
#include "robotname_msgs/srv/set_intake_mechanism.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

BT::PortsList SetIntakeMechanism::providedPorts()
{
    return {InputPort<float>("dribble"), InputPort<float>("lift")};
}

bool SetIntakeMechanism::setRequest(robotname_msgs::srv::SetIntakeMechanism::Request::SharedPtr& request)
{
    // use input ports to set A and B
    
    getInput("dribble", request->dribble);
    getInput("lift", request->lift);

    // must return true if we are ready to send the request
    return true;
}

NodeStatus SetIntakeMechanism::onResponseReceived(const robotname_msgs::srv::SetIntakeMechanism::Response::SharedPtr& response) 
  {
    if(response->status)
    {
      RCLCPP_INFO(node_->get_logger(), "[%s] send intake mechanism succesful", name().c_str());
      //RCLCPP_INFO(node_->get_logger(), "Sum: %ld", response->sum);
      return NodeStatus::SUCCESS;
    }
    RCLCPP_INFO(node_->get_logger(), "[%s] Failed send intake mechanism", name().c_str());
    return NodeStatus::FAILURE; 
  }

NodeStatus SetIntakeMechanism::onFailure(ServiceNodeErrorCode error) 
  {
    RCLCPP_ERROR(node_->get_logger(), "[%s] Error: %d", name().c_str(), error);
    return NodeStatus::FAILURE;
  }






