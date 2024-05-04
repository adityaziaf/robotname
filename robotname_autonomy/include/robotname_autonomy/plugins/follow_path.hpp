#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace BT;

class FollowPose: public RosTopicPubNode<geometry_msgs::msg::PoseStamped>
{
public:
  FollowPose(const std::string& name,
                const NodeConfig& conf,
                const RosNodeParams& params)
    : RosTopicPubNode<geometry_msgs::msg::PoseStamped>(name, conf, params)
  {}

  static BT::PortsList providedPorts();

  bool setMessage(geometry_msgs::msg::PoseStamped &msg) override;
 
};