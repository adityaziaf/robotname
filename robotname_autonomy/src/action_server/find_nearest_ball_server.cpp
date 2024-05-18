#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robotname_msgs/action/find_nearest_ball.hpp"
#include "behaviortree_ros2/bt_action_node.hpp"
#include "robotname_msgs/msg/detection_array.hpp"
#include "robotname_msgs/msg/detection.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"


class FindNearestBallServer : public rclcpp::Node
{
public:
  using FindNearestBall = robotname_msgs::action::FindNearestBall;
  using GoalHandleFindNearestBall = rclcpp_action::ServerGoalHandle<FindNearestBall>;

  explicit FindNearestBallServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("FindNearestBall_action_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<FindNearestBall>(
      this,
      "find_nearest_ball",
      std::bind(&FindNearestBallServer::handle_goal, this, _1, _2),
      std::bind(&FindNearestBallServer::handle_cancel, this, _1),
      std::bind(&FindNearestBallServer::handle_accepted, this, _1));

    _ball_sub = this->create_subscription<robotname_msgs::msg::DetectionArray>("/camera/objects/tracked", 1, std::bind(&FindNearestBallServer::handle_subscription, this,_1));

  }

private:
  rclcpp_action::Server<FindNearestBall>::SharedPtr action_server_;
  rclcpp::Subscription<robotname_msgs::msg::DetectionArray>::SharedPtr _ball_sub;
  robotname_msgs::msg::DetectionArray::SharedPtr last_msg;


  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const FindNearestBall::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with FindNearestBall object %s", goal->color.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFindNearestBall> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleFindNearestBall> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&FindNearestBallServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleFindNearestBall> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<FindNearestBall::Feedback>();
    auto result = std::make_shared<FindNearestBall::Result>();

    // rclcpp::Time deadline = get_clock()->now() + rclcpp::Duration::from_seconds( double(goal->msec_timeout) / 1000 );
    // int cycle = 0;
    bool ballfound = false;
    int result_id;

    while(rclcpp::ok() && !ballfound)
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
        bool object_founded;
        std::vector<std::pair<double, int>> target_objects;

        for(auto & object : last_msg->detections)
        {
          if (object.classname == goal->color) {
            object_founded = true;
            //RCLCPP_INFO(this->get_logger(), "Hit 0");
            double hypot_value = hypot(object.pose.pose.position.x, object.pose.pose.position.y);
            target_objects.push_back(std::make_pair(hypot_value, object.id));
            //RCLCPP_INFO(this->get_logger(), "Hit 1");
          }

        }

        if(object_founded)
        {
          //RCLCPP_INFO(this->get_logger(), "Hit 2.5");
          if(!target_objects.empty())
          {
            std::sort(target_objects.begin(), target_objects.end());RCLCPP_INFO(this->get_logger(), "Hit 1");
            result_id = target_objects.front().second;
            ballfound = true;
            RCLCPP_INFO(this->get_logger(), "Hit 3");
          }
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
      result->status = true;
      result->id = result_id;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }

  void handle_subscription(const robotname_msgs::msg::DetectionArray::SharedPtr msg)
  {
    last_msg = msg;
  }
};  // class FindNearestBallServer


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FindNearestBallServer>();

  rclcpp::spin(node);

  return 0;
}
