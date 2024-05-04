#pragma once

#include "behaviortree_ros2/bt_action_node.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"


using namespace BT;

class NavToPose: public RosActionNode<nav2_msgs::action::NavigateToPose>
{
public:
  NavToPose(const std::string& name,
              const NodeConfig& conf,
              const RosNodeParams& params)
    : RosActionNode<nav2_msgs::action::NavigateToPose>(name, conf, params)
  {}

  static BT::PortsList providedPorts();

  bool setGoal(Goal& goal) override;

  void onHalt() override;

  BT::NodeStatus onResultReceived(const WrappedResult& wr) override;

  virtual BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override;

  virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override;
};