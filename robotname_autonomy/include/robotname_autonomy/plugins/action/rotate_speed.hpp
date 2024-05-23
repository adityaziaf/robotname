#pragma once

#include "behaviortree_ros2/bt_action_node.hpp"
#include "robotname_msgs/action/set_speed.hpp"


using namespace BT;

class RotateSpeed: public RosActionNode<robotname_msgs::action::SetSpeed>
{
public:
  RotateSpeed(const std::string& name,
              const NodeConfig& conf,
              const RosNodeParams& params)
    : RosActionNode<robotname_msgs::action::SetSpeed>(name, conf, params)
  {}

  static BT::PortsList providedPorts();

  bool setGoal(Goal& goal) override;

  void onHalt() override;

  BT::NodeStatus onResultReceived(const WrappedResult& wr) override;

  virtual BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override;

  virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override;
};
