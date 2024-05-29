#include "rclcpp/rclcpp.hpp"
#include "robotname_msgs/srv/set_pneumatic.hpp"
#include "std_msgs/msg/bool.hpp"

#include <memory>

using namespace std::placeholders;

class SetPneumaticServer : public rclcpp::Node {
  public:
    SetPneumaticServer() : Node("SetPneumaticServer")
    {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_services_default;

      _service = this->create_service<robotname_msgs::srv::SetPneumatic>("set_pneumatic",
        std::bind(&SetPneumaticServer::handle_service, this, _1, _2),
        qos_profile);
      _publisher = this->create_publisher<std_msgs::msg::Bool>
        ("/set_pneumatic", rclcpp::QoS(10));
    }

    void handle_service(const std::shared_ptr<robotname_msgs::srv::SetPneumatic::Request> request, 
    const std::shared_ptr<robotname_msgs::srv::SetPneumatic::Response> response)
    {
      std_msgs::msg::Bool msg;
      msg.data = request->pneumatic;
      if(msg.data){
        RCLCPP_INFO(this->get_logger(), "[Set Pneumatic Server] Set Pneumatic to True");
      }else{
        RCLCPP_INFO(this->get_logger(), "[Set Pneumatic Server] Set Pneumatic to False");
      }
      _publisher->publish(msg);
      response->status = true;
    }

  private:
    rclcpp::Service<robotname_msgs::srv::SetPneumatic>::SharedPtr _service;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _publisher;
};



int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<SetPneumaticServer>node = std::make_shared<SetPneumaticServer>();

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Set Pneumatic Server Ready");

  rclcpp::spin(node);
  rclcpp::shutdown();
}