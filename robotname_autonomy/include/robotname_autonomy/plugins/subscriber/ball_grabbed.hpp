#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "std_msgs/msg/string.hpp"

using namespace BT;

class BallGrabbed: public RosTopicSubNode<std_msgs::msg::String>
{
public:
  BallGrabbed(const std::string& name,
                const NodeConfig& conf,
                const RosNodeParams& params)
    : RosTopicSubNode<std_msgs::msg::String>(name, conf, params)
  {}
  
  static BT::PortsList providedPorts();

  NodeStatus onTick(const std::shared_ptr<std_msgs::msg::String>& last_msg) override;
};

