#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robotname_msgs/action/follow.hpp"
#include "behaviortree_ros2/bt_action_node.hpp"
#include "robotname_msgs/msg/detection_array.hpp"
#include "robotname_msgs/msg/detection.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"


class FollowServer : public rclcpp::Node
{
public:
  using Follow = robotname_msgs::action::Follow;
  using GoalHandleFollow = rclcpp_action::ServerGoalHandle<Follow>;

  explicit FollowServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("FindNearestBall_action_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Follow>(
      this,
      "follow_ball",
      std::bind(&FollowServer::handle_goal, this, _1, _2),
      std::bind(&FollowServer::handle_cancel, this, _1),
      std::bind(&FollowServer::handle_accepted, this, _1));

    _ball_sub = this->create_subscription<robotname_msgs::msg::DetectionArray>("/camera/objects/tracked", 1, std::bind(&FollowServer::handle_subscription, this,_1));
    _goal_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose",10);
  }

private:
  rclcpp_action::Server<Follow>::SharedPtr action_server_;
  rclcpp::Subscription<robotname_msgs::msg::DetectionArray>::SharedPtr _ball_sub;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _goal_pub;
  robotname_msgs::msg::DetectionArray::SharedPtr last_msg;


  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const Follow::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with FindNearestBall object %s", goal->color.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFollow> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleFollow> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&FollowServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleFollow> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(20);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Follow::Feedback>();
    auto result = std::make_shared<Follow::Result>();

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
        bool objectfound = false;
        for(auto & object : last_msg->detections)
        {
          if (object.classname == goal->color) {
            if(object.id == goal->id)
            {
              objectfound = true;
              // geometry_msgs::msg::PoseStamped target;
              // target.set__header(object.pose.header);
              // target.pose.set__position(object.pose.pose.position);
              _goal_pub->publish(object.pose);
            }
          
          }
        }

        if(!objectfound)
        {
          state = false;
          geometry_msgs::msg::PoseStamped failed_goal;
          failed_goal.header.frame_id = "base_link";
          failed_goal.header.stamp = this->get_clock()->now();
          failed_goal.pose.position.x = 0;
          failed_goal.pose.position.y = 0;
          failed_goal.pose.position.z = 0;
          failed_goal.pose.orientation.x = 0;
          failed_goal.pose.orientation.y = 0;
          failed_goal.pose.orientation.z = 0;
          failed_goal.pose.orientation.w = 1;
          _goal_pub->publish(failed_goal);
          // result->status = false;
          // goal_handle->abort(result);
        }
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "Empty Topic Value");
      }

      // goal_handle->publish_feedback(feedback);
      // RCLCPP_INFO(this->get_logger(), "Publish feedback");

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok())
    {
      result->status = false;
      goal_handle->abort(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }

  void handle_subscription(const robotname_msgs::msg::DetectionArray::SharedPtr msg)
  {
    last_msg = msg;
  }
};  // class FollowServer


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FollowServer>();

  rclcpp::spin(node);

  return 0;
}