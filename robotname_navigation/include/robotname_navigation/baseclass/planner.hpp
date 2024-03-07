#pragma once
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose.hpp"

class planner
{
public:
planner(rclcpp::Node::SharedPtr node)  {}
virtual void initialize() {}
virtual nav_msgs::msg::Path createPlan(geometry_msgs::msg::PoseStamped start, geometry_msgs::msg::PoseStamped goal) {}

private:
rclcpp::Node::SharedPtr node_;
};