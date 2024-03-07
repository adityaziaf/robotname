// Copyright 2019 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <functional>
#include <memory>
#include <thread>
#include <chrono>
#include <iterator>

#include "robotname_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "nav_msgs/msg/odometry.hpp"

class PIDPathTracking : public rclcpp::Node
{
public:
  using PathTracking = robotname_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ServerGoalHandle<PathTracking>;

    std::string global_frame_, robot_frame_;
    double frequency_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    nav_msgs::msg::Odometry received_odom;

  explicit PIDPathTracking(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("pidcontroller", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<PathTracking>(
      this->get_node_base_interface(),
      this->get_node_clock_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "pidcontroller",
      std::bind(&PIDPathTracking::handleGoal, this, _1, _2),
      std::bind(&PIDPathTracking::handleCancel, this, _1),
      std::bind(&PIDPathTracking::handleAccepted, this, _1));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    this->declare_parameter("global_frame","map");
    this->declare_parameter("robot_frame","base_link");
    this->declare_parameter("frequency", 50.0);

    this->get_parameter("global_frame",global_frame_);
    this->get_parameter("robot_frame",robot_frame_);
    this->get_parameter("frequency", frequency_);
    
  }

private:
  rclcpp_action::Server<PathTracking>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const PathTracking::Goal> goal)
  {
    (void)uuid;
    if(goal->path.header.frame_id != global_frame_)
    {
        RCLCPP_INFO(this->get_logger(), "path frame differentfrom global_frame");
        rclcpp_action::GoalResponse::REJECT;
    }
    auto start = goal->path.poses.begin()->pose;
    auto end = goal->path.poses.end()->pose;
    RCLCPP_INFO(this->get_logger(), "Received goal request from %lf:%lf:%lf to %lf:%lf:%lf",
    start.position.x, start.position.y, start.orientation.z, end.position.x, end.position.y, end.orientation.z);

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandle> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(frequency_);

    const auto goal = goal_handle->get_goal();
    // auto feedback = std::make_shared<PathTracking::Feedback>();
    auto path = goal->path;
    
    //auto transformed_path = transformGlobalToLocal(goal->path);
    auto result = std::make_shared<PathTracking::Result>();

    bool goalstatus = false;

    while(goalstatus)
    {
      auto it = std::next(path.poses.begin(), 1);
      
      auto t_translation_x = it->pose.position.x - received_odom.pose.pose.position.x;
      auto t_translation_y = it->pose.position.y - received_odom.pose.pose.position.y;
      auto t_heading = it->pose.orientation.z - received_odom.pose.pose.orientation.z;

      auto t_translation = std::hypot(t_translation_x,t_translation_y);
      auto t_theta = std::atan2(t_translation_y, t_translation_x);

      auto pid_target_translation = PID_translation.compute(t_translation);
      auto pi_target_heading = PID_heading.compute(t_heading);

      
    }
    // for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
    //   // Check if there is a cancel request
    //   if (goal_handle->is_canceling()) {
    //     result->sequence = sequence;
    //     goal_handle->canceled(result);
    //     RCLCPP_INFO(this->get_logger(), "Goal Canceled");
    //     return;
    //   }
    //   // Update sequence
    //   sequence.push_back(sequence[i] + sequence[i - 1]);
    //   // Publish feedback
    //   goal_handle->publish_feedback(feedback);
    //   RCLCPP_INFO(this->get_logger(), "Publish Feedback");

    //   loop_rate.sleep();
    // }

    // Check if goal is done
    // if (rclcpp::ok()) {
    //   result->sequence = sequence;
    //   goal_handle->succeed(result);
    //   RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
    // }
  }

  void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&PIDPathTracking::execute, this, _1), goal_handle}.detach();
  }

  nav_msgs::msg::Path transformGlobalToLocal(nav_msgs::msg::Path path)
  {
    nav_msgs::msg::Path transformed_path;
    transformed_path.header.frame_id = path.header.frame_id;
    transformed_path.header.stamp = path.header.stamp;
    for(auto &point: path.poses)
    {
      geometry_msgs::msg::PoseStamped transformed_point=
                tf_buffer_->transform(point, global_frame_,tf2::durationFromSec(1));
            transformed_path.poses.push_back(transformed_point);
    }
    return transformed_path;
  }
  
  void odomCallback(nav_msgs::msg::Odometry::SharedPtr odom)
  {
    received_odom = *odom;
  }
};  // class MinimalActionServer

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<PIDPathTracking>();

  rclcpp::spin(action_server);

  rclcpp::shutdown();
  return 0;
}