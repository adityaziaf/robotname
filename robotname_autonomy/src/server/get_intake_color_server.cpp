#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/string.hpp"

#include <memory>

using namespace std::placeholders;

class GetIntakeColor : public rclcpp::Node {
  public:
    GetIntakeColor() : Node("GetIntakeColor")
    {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_default;

      _service = this->create_service<std_srvs::srv::Trigger>("get_intake_color",
        std::bind(&GetIntakeColor::handle_service, this, _1, _2),
        qos_profile);
      _subscriber = this->create_subscription<std_msgs::msg::String>
        ("/intake/detectedcolor", rclcpp::QoS(1),
            std::bind(&GetIntakeColor::detection_callback, this, _1));
    }

    void handle_service(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, 
    const std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      if(_last_msg != nullptr)
      {
        (void)request;
        response->message = _last_msg->data;
        response->success = true;
        
      }
    }

    void detection_callback(const std_msgs::msg::String::SharedPtr msg)
    {
      _last_msg = msg;
    }

  private:
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _service;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _subscriber;
    std_msgs::msg::String::SharedPtr _last_msg;

};



int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<GetIntakeColor>node = std::make_shared<GetIntakeColor>();

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Find Nearest Ball Server Ready");

  rclcpp::spin(node);
  rclcpp::shutdown();
}