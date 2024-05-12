#include "rclcpp/rclcpp.hpp"
#include "robotname_msgs/srv/follow_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <memory>

using namespace std::placeholders;

class FollowPose : public rclcpp::Node {
  public:
    FollowPose() : Node("FollowPose")
    {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_services_default;

      _service = this->create_service<robotname_msgs::srv::FollowPose>("follow_path",
        std::bind(&FollowPose::handle_service, this, _1, _2),
        qos_profile);
      _publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>
        ("/goal_pose", rclcpp::QoS(1));
    }

    void handle_service(const std::shared_ptr<robotname_msgs::srv::FollowPose::Request> request, 
    const std::shared_ptr<robotname_msgs::srv::FollowPose::Response> response)
    {
      _publisher->publish(request->goal);
      response->status = true;
    }

  private:
    rclcpp::Service<robotname_msgs::srv::FollowPose>::SharedPtr _service;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _publisher;
};



int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<FollowPose>node = std::make_shared<FollowPose>();

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "FollowPose Server Ready");

  rclcpp::spin(node);
  rclcpp::shutdown();
}