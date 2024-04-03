// Navigation Related Behaviors

#include "robotname_autonomy/plugins/go_through_poses.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// GOTOPOSE
// Wrapper behavior around the `navigate_to_pose` action client,
// whose status reflects the status of the ROS action.
GoThroughPoses::GoThroughPoses(const std::string& name, const BT::NodeConfig& config,
                   rclcpp::Node::SharedPtr node_ptr) :
    BT::StatefulActionNode(name, config), node_ptr_{node_ptr} {}

BT::NodeStatus GoThroughPoses::onStart() {
    // Validate that a node exists
    if (!node_ptr_) {
        std::cout << "ROS2 node not registered via init() method" << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    // wait for the action service available 
     auto res = getInput<Poses>("goals");
      if( !res )
      {
        throw BT::RuntimeError("error reading port [target]:", res.error());
      }
      Poses target_poses = res.value();

    // Set up the action client
    using namespace std::placeholders;

    
    auto send_goal_options = 
        rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();
    send_goal_options.result_callback =
        std::bind(&GoThroughPoses::result_callback, this, _1);
    client_ptr_ = rclcpp_action::create_client<NavigateThroughPoses>(
      node_ptr_, "/navigate_through_poses");

    if (!client_ptr_->wait_for_action_server(std::chrono::seconds(5))) {
        std::cout << "[" << this->name() << "] Action Server Unavailable" << std::endl;
        return BT::NodeStatus::FAILURE;
    }
    // Package up the the goal
    auto goal_msg = NavigateThroughPoses::Goal();

    for(auto &pose: target_poses)
    {
        geometry_msgs::msg::PoseStamped goal_pose;

        goal_pose.header.frame_id = "map";
        goal_pose.pose.position.x = pose.x;
        goal_pose.pose.position.y = pose.y;
        tf2::Quaternion q;
        q.setRPY(0, 0, pose.theta);
        q.normalize();
        goal_pose.pose.orientation = tf2::toMsg(q);

        goal_msg.poses.push_back(goal_pose);
    }

    // Send the navigation action goal
    done_flag_ = false;
    auto goal_handle_future = client_ptr_->async_send_goal(goal_msg, send_goal_options);

    std::cout << "[" << this->name() << "] Sent goal message" << std::endl;
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoThroughPoses::onRunning() {
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

BT::PortsList GoThroughPoses::providedPorts() {
    return { BT::InputPort<Poses>("goals") };
}

void GoThroughPoses::result_callback(const GoalHandleNav::WrappedResult& result) {
    // If there is a result, we consider navigation completed and save the
    // result code to be checked in the `onRunning()` method.
    if (result.result) {
        done_flag_ = true;
        nav_result_ = result.code;
    }
}
