// Navigation behaviors for TurtleBot3

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "yaml-cpp/yaml.h"

// Struct to keep location pose data
struct Pose{
  double x;
  double y;
  double theta;
};

// Go to a target location (wraps around `navigate_to_pose` action).
class goToPose : public BT::StatefulActionNode
{
  public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    bool done_flag_;
    rclcpp_action::ResultCode nav_result_;
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;

    // Method overrides
    goToPose(const std::string& name, const BT::NodeConfig& config,
             rclcpp::Node::SharedPtr node_ptr);
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override {};
    static BT::PortsList providedPorts();

    // Action client callbacks
    void result_callback(const GoalHandleNav::WrappedResult& result);
};