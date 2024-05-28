#pragma once

#include "behaviortree_ros2/bt_action_node.hpp"
#include "robotname_msgs/action/setup_menu.hpp"

using namespace BT;

class SetupMenu: public RosActionNode<robotname_msgs::action::SetupMenu>
{
public:
  SetupMenu(const std::string& name,
              const NodeConfig& conf,
              const RosNodeParams& params)
    : RosActionNode<robotname_msgs::action::SetupMenu>(name, conf, params)
  {}

  static BT::PortsList providedPorts();

  bool setGoal(Goal& goal) override;

  void onHalt() override;

  BT::NodeStatus onResultReceived(const WrappedResult& wr) override;

  virtual BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override;

  virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override;
};
