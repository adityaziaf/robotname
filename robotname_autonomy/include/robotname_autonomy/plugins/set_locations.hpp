// Navigation behaviors for TurtleBot3

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "yaml-cpp/yaml.h"
#include "robotname_autonomy/utils.hpp"



// YAML parsing template specialization for the Pose type, which is used to parse locations directly
namespace YAML {
    template<>
    struct convert<Pose> {
        static bool decode(const Node& node, Pose& pose) {
            if (!node.IsSequence() || node.size() != 3) {
                return false;
            }
            pose.x = node[0].as<double>();
            pose.y = node[1].as<double>();
            pose.theta = node[2].as<double>();
            return true;
        }
    };
}

// Sets number of locations from list.
class SetLocations : public BT::SyncActionNode
{
  public:
    SetLocations(const std::string& name, const BT::NodeConfig& config);
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();
};
