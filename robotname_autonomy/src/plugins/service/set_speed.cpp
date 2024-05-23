#include "behaviortree_ros2/bt_service_node.hpp"
#include "robotname_autonomy/plugins/service/set_speed.hpp"
#include "robotname_autonomy/utils.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
#include "geometry_msgs/msg/twist.hpp"

BT::PortsList SetSpeed::providedPorts()
{
    return {InputPort<Pose>("speed")};
}

bool SetSpeed::setRequest(robotname_msgs::srv::SetSpeed::Request::SharedPtr& request)
{
    // use input ports to set A and B
    Pose speed;
    getInput("speed", speed);
    geometry_msgs::msg::Twist tw;
    tw.linear.x = speed.x;
    tw.linear.y = speed.y;
    tw.angular.z = speed.theta;

    request->set__speed(tw);

    // must return true if we are ready to send the request
    return true;
}

NodeStatus SetSpeed::onResponseReceived(const robotname_msgs::srv::SetSpeed::Response::SharedPtr& response) 
  {
    if(response->status)
    {
      RCLCPP_INFO(node_->get_logger(), "[%s] send speed succesful", name().c_str());
      //RCLCPP_INFO(node_->get_logger(), "Sum: %ld", response->sum);
      return NodeStatus::SUCCESS;
    }
    RCLCPP_INFO(node_->get_logger(), "[%s] Failed send speed", name().c_str());
    return NodeStatus::FAILURE; 
  }

NodeStatus SetSpeed::onFailure(ServiceNodeErrorCode error) 
  {
    RCLCPP_ERROR(node_->get_logger(), "[%s] Error: %d", name().c_str(), error);
    return NodeStatus::FAILURE;
  }






