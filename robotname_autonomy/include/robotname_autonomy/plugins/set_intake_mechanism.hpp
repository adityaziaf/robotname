#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include <std_msgs/msg/float32_multi_array.hpp>

using namespace BT;

class SetIntakeMechanism: public RosTopicPubNode<std_msgs::msg::Float32MultiArray>
{
public:
  SetIntakeMechanism(const std::string& name,
                const NodeConfig& conf,
                const RosNodeParams& params)
    : RosTopicPubNode<std_msgs::msg::Float32MultiArray>(name, conf, params)
  {}

  static BT::PortsList providedPorts();

  bool setMessage(std_msgs::msg::Float32MultiArray &msg) override;
 
};