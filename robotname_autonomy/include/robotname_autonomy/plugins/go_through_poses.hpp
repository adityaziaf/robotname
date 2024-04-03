// Navigation behaviors for TurtleBot3

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "yaml-cpp/yaml.h"
#include "robotname_autonomy/utils.hpp"


// Go to a target location (wraps around `navigate_to_pose` action).
class GoThroughPoses : public BT::StatefulActionNode
{
  public:
    using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

    bool done_flag_;
    rclcpp_action::ResultCode nav_result_;
    rclcpp::Node::SharedPtr node_ptr_;
    rclcpp_action::Client<NavigateThroughPoses>::SharedPtr client_ptr_;

    // Method overrides
    GoThroughPoses(const std::string& name, const BT::NodeConfig& config,
             rclcpp::Node::SharedPtr node_ptr);
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override {};
    static BT::PortsList providedPorts();

    // Action client callbacks
    void result_callback(const GoalHandleNav::WrappedResult& result);
};
