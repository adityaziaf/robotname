#pragma once

#include "behaviortree_ros2/bt_action_node.hpp"
#include "robotname_msgs/action/get_intake_proximity_array.hpp"


using namespace BT;

class GetIntakeProximityArray: public RosActionNode<robotname_msgs::action::GetIntakeProximityArray>
{
public:
  GetIntakeProximityArray(const std::string& name,
              const NodeConfig& conf,
              const RosNodeParams& params)
    : RosActionNode<robotname_msgs::action::GetIntakeProximityArray>(name, conf, params)
  {}

  static BT::PortsList providedPorts();

  bool setGoal(Goal& goal) override;

  void onHalt() override;

  BT::NodeStatus onResultReceived(const WrappedResult& wr) override;

  virtual BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override;

  virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override;
};
