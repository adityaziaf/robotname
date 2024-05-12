#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/bool.hpp"
#include <memory>

using namespace std::placeholders;

class ResetBallGrabbed : public rclcpp::Node {
  public:
    ResetBallGrabbed() : Node("ResetBallGrabbed")
    {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_services_default;

      _service = this->create_service<std_srvs::srv::SetBool>("reset_ball_grabbed",
        std::bind(&ResetBallGrabbed::handle_service, this, _1, _2),
        qos_profile);
      _publisher = this->create_publisher<std_msgs::msg::Bool>
        ("/ballgrabbed", rclcpp::QoS(10));
    }

    void handle_service(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, 
    const std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
      (void)request;

      std_msgs::msg::Bool data;
      data.data = false;
      _publisher->publish(data);
      response->success = true;
    }

  private:
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr _service;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _publisher;
};



int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<ResetBallGrabbed>node = std::make_shared<ResetBallGrabbed>();

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "reset ballgrabbed Server Ready");

  rclcpp::spin(node);
  rclcpp::shutdown();
}