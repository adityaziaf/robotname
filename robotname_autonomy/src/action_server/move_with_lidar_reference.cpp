#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robotname_msgs/action/set_speed.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "behaviortree_ros2/bt_action_node.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <memory>
#include "tf2/utils.h"

class MoveWithLidarReference : public rclcpp::Node
{
public:
  using SetSpeed = robotname_msgs::action::SetSpeed;
  using GoalHandleSetSpeed = rclcpp_action::ServerGoalHandle<SetSpeed>;

  explicit MoveWithLidarReference(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("move_with_lidar_reference", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<SetSpeed>(
      this,
      "move_with_lidar_reference",
      std::bind(&MoveWithLidarReference::handle_goal, this, _1, _2),
      std::bind(&MoveWithLidarReference::handle_cancel, this, _1),
      std::bind(&MoveWithLidarReference::handle_accepted, this, _1));
    
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    twist_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",10);
    pose_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 1, std::bind(&MoveWithLidarReference::handle_subscription, this,_1));
  }

private:
  rclcpp_action::Server<SetSpeed>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr pose_sub;
  sensor_msgs::msg::LaserScan::SharedPtr last_msg;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const SetSpeed::Goal> goal)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleSetSpeed> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleSetSpeed> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&MoveWithLidarReference::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleSetSpeed> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(30);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<SetSpeed::Feedback>();
    auto result = std::make_shared<SetSpeed::Result>();

    bool state = true;

    while(rclcpp::ok() && state)
    {
      if (goal_handle->is_canceling())
      {
        result->status = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      if(last_msg)
      {
        try {
        geometry_msgs::msg::TransformStamped transform =
        tf_buffer_->lookupTransform(goal->parent_frame,
                                    goal->child_frame_id,
                                    tf2::TimePointZero);
        
        sensor_msgs::msg::LaserScan in, out;
        tf2::doTransform<sensor_msgs::msg::LaserScan>(*last_msg, out, transform); 

        
        // double target = last_msg->



      } catch (const tf2::TransformException &ex) {
        RCLCPP_INFO(this->get_logger(), "Could not transform");
      }

      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "Empty Topic Value");
      }
      
      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok())
    {
      result->status = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }

  void handle_subscription(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    last_msg = msg;
  }

};  // class MoveWithLidarReference


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveWithLidarReference>();

  rclcpp::spin(node);

  return 0;
}
