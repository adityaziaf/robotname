#include "rclcpp/rclcpp.hpp"
#include "robotname_msgs/srv/rotate.hpp"
#include "std_msgs/msg/bool.hpp"
#include <memory>
#include "behaviortree_ros2/plugins.hpp"
#include "robotname_autonomy/utils.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::placeholders;

class RotateServer : public rclcpp::Node {
  public:
    RotateServer() : Node("RotateServer")
    {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_services_default;

      _service = this->create_service<robotname_msgs::srv::Rotate>("rotate_server",
        std::bind(&RotateServer::handle_service, this, _1, _2),
        qos_profile);
    }

    void handle_service(const std::shared_ptr<robotname_msgs::srv::Rotate::Request> request, 
    const std::shared_ptr<robotname_msgs::srv::Rotate::Response> response)
    {
        geometry_msgs::msg::PoseStamped point;
        point = request->pose;
        tf2::Quaternion q;
        q.setRPY(0, 0, request->angle);
        q.normalize();
        point.pose.set__orientation(tf2::toMsg(q));
        response->set__pose(point);
        response->status = true;
        return;
    }

  private:
    rclcpp::Service<robotname_msgs::srv::Rotate>::SharedPtr _service;
};



int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<RotateServer>node = std::make_shared<RotateServer>();

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "rotate Server Ready");

  rclcpp::spin(node);
  rclcpp::shutdown();
}