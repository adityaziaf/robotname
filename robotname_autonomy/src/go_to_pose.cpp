// Navigation Related Behaviors

#include "robotname_autonomy/go_to_pose.hpp"

#include <random>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


// GOTOPOSE
// Wrapper behavior around the `navigate_to_pose` action client,
// whose status reflects the status of the ROS action.
goToPose::goToPose(const std::string& name, const BT::NodeConfig& config,
                   rclcpp::Node::SharedPtr node_ptr) :
    BT::StatefulActionNode(name, config), node_{node_ptr} {}

BT::NodeStatus goToPose::onStart() {
    // Validate that a node exists
    if (!node_) {
        std::cout << "ROS2 node not registered via init() method" << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    // Read the YAML file
    BT::Expected<std::string> loc = getInput<std::string>("loc");

    auto location_poses = getInput<std::map<std::string,Pose>>("loc_poses");
    if (!location_poses){
        std::cerr << "Couldn't get loc_poses!" << std::endl;
        return BT::NodeStatus::FAILURE;
    }
    auto target_loc = getInput<std::string>("loc");
    if (!target_loc) {
        std::cerr << "Couldn't get target loc!" << std::endl;
        return BT::NodeStatus::FAILURE;
    }
    auto target_pose = location_poses.value().at(target_loc.value());
    
    // Set up the action client
    using namespace std::placeholders;
    auto send_goal_options = 
        rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback =
        std::bind(&goToPose::result_callback, this, _1);
    client_ptr_ = rclcpp_action::create_client<NavigateToPose>(
      node_, "/navigate_to_pose");

    // Package up the the goal
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.pose.position.x = target_pose.x;
    goal_msg.pose.pose.position.y = target_pose.y;
    tf2::Quaternion q;
    q.setRPY(0, 0, target_pose.theta);
    q.normalize();
    goal_msg.pose.pose.orientation = tf2::toMsg(q);

    // Send the navigation action goal.
    done_flag_ = false;
    client_ptr_->async_send_goal(goal_msg, send_goal_options);
    std::cout << "[" << this->name() << "] Sent goal message" << std::endl;
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus goToPose::onRunning() {
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
        return BT::NodeStatus::RUNNING;
    }
}

BT::PortsList goToPose::providedPorts() {
    return { BT::InputPort<std::string>("loc"),
             BT::InputPort<std::map<std::string, Pose>>("loc_poses") };
}

void goToPose::result_callback(const GoalHandleNav::WrappedResult& result) {
    // If there is a result, we consider navigation completed and save the
    // result code to be checked in the `onRunning()` method.
    if (result.result) {
        done_flag_ = true;
        nav_result_ = result.code;
    }
}