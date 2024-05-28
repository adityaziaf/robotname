#include "rclcpp/rclcpp.hpp"
#include "robotname_msgs/srv/set_speed.hpp"
#include "geometry_msgs/msg/twist.h"

#include <memory>

using namespace std::placeholders;

class SetSpeed : public rclcpp::Node {
  public:
    SetSpeed() : Node("SetSpeed")
    {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_services_default;

      _service = this->create_service<robotname_msgs::srv::SetSpeed>("set_speed",
        std::bind(&SetSpeed::handle_service, this, _1, _2),
        qos_profile);
      _publisher = this->create_publisher<geometry_msgs::msg::Twist>
        ("/cmd_vel", rclcpp::QoS(1));
    }

    void handle_service(const std::shared_ptr<robotname_msgs::srv::SetSpeed::Request> request, 
    const std::shared_ptr<robotname_msgs::srv::SetSpeed::Response> response)
    {
      _publisher->publish(request->speed);
      response->status = true;
    }

  private:
    rclcpp::Service<robotname_msgs::srv::SetSpeed>::SharedPtr _service;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _publisher;
};



int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<SetSpeed>node = std::make_shared<SetSpeed>();

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Set Speed Server Ready");

  rclcpp::spin(node);
  rclcpp::shutdown();
}