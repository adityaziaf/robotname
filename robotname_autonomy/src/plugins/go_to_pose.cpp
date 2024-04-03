// Navigation Related Behaviors

#include "robotname_autonomy/plugins/go_to_pose.hpp"

#include <random>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// GOTOPOSE
// Wrapper behavior around the `navigate_to_pose` action client,
// whose status reflects the status of the ROS action.
GoToPose::GoToPose(const std::string& name, const BT::NodeConfig& config,
                   rclcpp::Node::SharedPtr node_ptr) :
    BT::StatefulActionNode(name, config), node_ptr_{node_ptr} {}

BT::NodeStatus GoToPose::onStart() {
    // Validate that a node exists
    if (!node_ptr_) {
        std::cout << "ROS2 node not registered via init() method" << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    // wait for the action service available 
     auto res = getInput<Pose>("goal");
      if( !res )
      {
        throw BT::RuntimeError("error reading port [target]:", res.error());
      }
      Pose target_pose = res.value();
    // Set up the action client
    using namespace std::placeholders;

    auto send_goal_options = 
        rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback =
        std::bind(&GoToPose::result_callback, this, _1);
    client_ptr_ = rclcpp_action::create_client<NavigateToPose>(
      node_ptr_, "/navigate_to_pose");

    if (!client_ptr_->wait_for_action_server(std::chrono::seconds(5))) {
        std::cout << "[" << this->name() << "] Action Server Unavailable" << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    // Package up the the goal
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.pose.position.x = target_pose.x;
    goal_msg.pose.pose.position.y = target_pose.y;
    tf2::Quaternion q;
    q.setRPY(0, 0, target_pose.theta);
    q.normalize();
    goal_msg.pose.pose.orientation = tf2::toMsg(q);

    // Send the navigation action goal
    done_flag_ = false;
    auto goal_handle_future = client_ptr_->async_send_goal(goal_msg, send_goal_options);

    std::cout << "[" << this->name() << "] Sent goal message" << std::endl;
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoToPose::onRunning() {
    // If there is a result, we can check the status of the action directly.
    // Otherwise, the action is still running.
    if (done_flag_) {
        if (nav_result_ == rclcpp_action::ResultCode::SUCCEEDED) {
            std::cout << "[" << this->name() << "] Goal reached" << std::endl;
            return BT::NodeStatus::SUCCESS;   
        } else {
            std::cout << "[" << this->name() << "] Failed to reach goal" << std::endl;
            return BT::NodeStatus::FAILURE;   
        }
    } else {
        std::cout << "[" << this->name() << "] Running" << std::endl;
        return BT::NodeStatus::RUNNING;
        
    }
}

BT::PortsList GoToPose::providedPorts() {
    return { BT::InputPort<Pose>("goal") };
}

void GoToPose::result_callback(const GoalHandleNav::WrappedResult& result) {
    // If there is a result, we consider navigation completed and save the
    // result code to be checked in the `onRunning()` method.
    if (result.result) {
        done_flag_ = true;
        nav_result_ = result.code;
    }
}
