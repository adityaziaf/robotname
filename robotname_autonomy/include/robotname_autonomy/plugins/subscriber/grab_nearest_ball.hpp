#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "robotname_msgs/msg/detection_array.hpp"

using namespace BT;

class GrabNearestBall: public RosTopicSubNode<robotname_msgs::msg::DetectionArray>
{
public:
  GrabNearestBall(const std::string& name,
                const NodeConfig& conf,
                const RosNodeParams& params)
    : RosTopicSubNode<robotname_msgs::msg::DetectionArray>(name, conf, params)
  {}

  static BT::PortsList providedPorts();

  NodeStatus onTick(const std::shared_ptr<robotname_msgs::msg::DetectionArray>& last_msg) override;
};

