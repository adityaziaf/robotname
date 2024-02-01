#pragma once

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "robotname_msgs/msg/detection_array.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

// Look for an object of a particular color
class findObject : public BT::StatefulActionNode
{
  public:
    // Method overrides
    findObject(const std::string& name, const BT::NodeConfiguration& config,
                  rclcpp::Node::SharedPtr node_ptr);
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
    static BT::PortsList providedPorts();

    //subscriber callback
    void detection_callback(const robotname_msgs::msg::DetectionArray::SharedPtr msg);

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<robotname_msgs::msg::DetectionArray>::SharedPtr objects_subs_;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr pos_subs_;
    robotname_msgs::msg::DetectionArray::SharedPtr latest_msg;
    bool received_image_;
    std::string objectname_;
    
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::string t_frame, s_frame;

};