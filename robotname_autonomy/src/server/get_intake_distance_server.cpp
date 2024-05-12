#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/float32.hpp"
#include "robotname_msgs/srv/get_intake_distance.hpp"

#include <memory>

using namespace std::placeholders;

class GetIntakeDistance : public rclcpp::Node {
  public:
    GetIntakeDistance() : Node("GetIntakeDistance")
    {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_services_default;

      _service = this->create_service<robotname_msgs::srv::GetIntakeDistance>("get_intake_distance",
        std::bind(&GetIntakeDistance::handle_service, this, _1, _2),
        qos_profile);
      _subscriber = this->create_subscription<std_msgs::msg::Float32>
        ("/proximity", rclcpp::QoS(1),
            std::bind(&GetIntakeDistance::detection_callback, this, _1));
    }

    void handle_service(const std::shared_ptr<robotname_msgs::srv::GetIntakeDistance::Request> request, 
    const std::shared_ptr<robotname_msgs::srv::GetIntakeDistance::Response> response)
    {
      if(_last_msg != nullptr)
      {
        (void)request;
        response->distance = _last_msg->data;
        response->status = true;
        
      }
    }

    void detection_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
      _last_msg = msg;
    }

  private:
    rclcpp::Service<robotname_msgs::srv::GetIntakeDistance>::SharedPtr _service;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _subscriber;
    std_msgs::msg::Float32::SharedPtr _last_msg;

};



int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<GetIntakeDistance>node = std::make_shared<GetIntakeDistance>();

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Get intake Distance Server Ready");

  rclcpp::spin(node);
  rclcpp::shutdown();
}