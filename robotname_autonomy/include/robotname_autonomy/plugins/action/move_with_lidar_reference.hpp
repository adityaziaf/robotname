#pragma once

#include "behaviortree_ros2/bt_action_node.hpp"
#include "robotname_msgs/action/move_with_lidar_reference.hpp"


using namespace BT;

class MoveWithLidarReference: public RosActionNode<robotname_msgs::action::MoveWithLidarReference>
{
public:
  MoveWithLidarReference(const std::string& name,
              const NodeConfig& conf,
              const RosNodeParams& params)
    : RosActionNode<robotname_msgs::action::MoveWithLidarReference>(name, conf, params)
  {}

  static BT::PortsList providedPorts();

  bool setGoal(Goal& goal) override;

  void onHalt() override;

  BT::NodeStatus onResultReceived(const WrappedResult& wr) override;

  virtual BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override;

  virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override;
};
