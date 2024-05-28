#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robotname_msgs/action/set_speed.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "behaviortree_ros2/bt_action_node.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "robotname_msgs/msg/detection_array.hpp"
#include "tf2/utils.h"

class RotateSpeedServer : public rclcpp::Node
{
public:
  using SetSpeed = robotname_msgs::action::SetSpeed;
  using GoalHandleSetSpeed = rclcpp_action::ServerGoalHandle<SetSpeed>;

  explicit RotateSpeedServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("SetSpeed_action_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<SetSpeed>(
      this,
      "rotate_speed",
      std::bind(&RotateSpeedServer::handle_goal, this, _1, _2),
      std::bind(&RotateSpeedServer::handle_cancel, this, _1),
      std::bind(&RotateSpeedServer::handle_accepted, this, _1));
    
    twist_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",10);
    pose_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/amcl_pose", 1, std::bind(&RotateSpeedServer::handle_subscription, this,_1));
  }

private:
  rclcpp_action::Server<SetSpeed>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub;
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr last_msg;

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

  double angleWrap(double angle) {
    
    if(angle >= M_PI)
    {
      angle -= 2*M_PI;
    }

    else if(angle < -M_PI)
    {
      angle += 2*M_PI;
    }

    return angle;
}

  void handle_accepted(const std::shared_ptr<GoalHandleSetSpeed> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&RotateSpeedServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleSetSpeed> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(100);
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
        double theta = tf2::getYaw(last_msg->pose.pose.orientation);

        double target = angleWrap(goal->angle - theta);

        geometry_msgs::msg::Twist tw_msg;

        if(target >= 0)
        {   
          tw_msg.angular.z = goal->speed;

          if(target <= 0.1)
          {
            tw_msg.angular.z = 0;
            state = false;
          }
        }

        else if (target < 0)
        {
          tw_msg.angular.z = -goal->speed;

          if(target >= -0.1)
          {
            tw_msg.angular.z = 0.0;
            state = false;
          }
        }
        
        twist_pub->publish(tw_msg);

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

  void handle_subscription(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    last_msg = msg;
  }

};  // class RotateSpeedServer


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RotateSpeedServer>();

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Rotate Speed Server Ready");

  rclcpp::spin(node);

  return 0;
}
