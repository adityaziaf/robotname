#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robotname_msgs/action/move_with_lidar_reference.hpp"
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
  using MoveWithLidar = robotname_msgs::action::MoveWithLidarReference;
  using GoalHandleMoveWithLidar = rclcpp_action::ServerGoalHandle<MoveWithLidar>;

  explicit MoveWithLidarReference(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("move_with_lidar_reference", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<MoveWithLidar>(
      this,
      "move_with_lidar_reference",
      std::bind(&MoveWithLidarReference::handle_goal, this, _1, _2),
      std::bind(&MoveWithLidarReference::handle_cancel, this, _1),
      std::bind(&MoveWithLidarReference::handle_accepted, this, _1));
    
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    twist_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",10);
    pose_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan/transform", 1, std::bind(&MoveWithLidarReference::handle_subscription, this,_1));

  }

private:
  rclcpp_action::Server<MoveWithLidar>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr pose_sub;
  sensor_msgs::msg::LaserScan::SharedPtr last_msg;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const MoveWithLidar::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with MoveWithLidarReference %lf", goal->y);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleMoveWithLidar> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleMoveWithLidar> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&MoveWithLidarReference::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleMoveWithLidar> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(30);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<MoveWithLidar::Feedback>();
    auto result = std::make_shared<MoveWithLidar::Result>();

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
        //get the median from 5 measurements along x and y axis
        // assume 1.57 paralel to y axis
        // assume 3.14 - 0.2 paralel to - x axis
        int index_along_y = round((-M_PI/2.0 - last_msg->angle_min) / last_msg->angle_increment);
        //int index_along_x = round((M_PI - 0.2 - last_msg->angle_min) / last_msg->angle_increment);

        //std::vector<double> x_container;
        std::vector<double> y_container;

        //std::copy(last_msg->ranges.begin() + index_along_x, last_msg->ranges.begin() + index_along_x + 5, std::back_inserter(x_container));
        std::copy(last_msg->ranges.begin() + index_along_y, last_msg->ranges.begin() + index_along_y + 5, std::back_inserter(y_container));

        //double x_median = calculateMedian(x_container);
        double y_median = calculateMedian(y_container);
        // try set speed with some goal target tolerance

        double target_position = goal->y - y_median;
        geometry_msgs::msg::Twist msg;
        msg.linear.x = -0.4;

        if(target_position >= 0)
        {
          msg.linear.y = goal->speed;
          if(target_position <= 0.05) 
          {
            msg.linear.y = 0.0; //goal tolerance
            msg.linear.x = 0.0;
            state = false;
          }
        }
        else 
        {
          msg.linear.y = -goal->speed;
          if(target_position >= -0.05) 
          {
            msg.linear.y = 0.0; // goal tolerance
            msg.linear.x = 0.0;
            state = false;
          }
        }
        twist_pub->publish(msg);

        RCLCPP_INFO(this->get_logger(), "Y Median: %lf", y_median);
            
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

  double calculateMedian(std::vector<double>& measurements) {
    // Sort the measurements
    std::sort(measurements.begin(), measurements.end());

    // Find the number of measurements
    int n = measurements.size();

    // Calculate the median
    if (n % 2 == 1) {
        return measurements[(n - 1) / 2];
    } else {
        return (measurements[n / 2] + measurements[(n / 2) - 1]) / 2.0;
    }
  }


};  // class MoveWithLidarReference


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveWithLidarReference>();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Move With Lidar Reference Server Ready");

  rclcpp::spin(node);

  return 0;
}
