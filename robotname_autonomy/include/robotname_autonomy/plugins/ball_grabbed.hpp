#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace BT;

class BallGrabbed: public RosTopicSubNode<std_msgs::msg::Bool>
{
public:
  BallGrabbed(const std::string& name,
                const NodeConfig& conf,
                const RosNodeParams& params)
    : RosTopicSubNode<std_msgs::msg::Bool>(name, conf, params)
  {}

  NodeStatus onTick(const std::shared_ptr<std_msgs::msg::Bool>& last_msg) override;
};

