#include "rclcpp/rclcpp.hpp"
#include "robotname_msgs/srv/set_intake_mechanism.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include <memory>

using namespace std::placeholders;

class SetIntakeMechanismServer : public rclcpp::Node {
  public:
    SetIntakeMechanismServer() : Node("SetIntakeMechanismServer")
    {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_services_default;

      _service = this->create_service<robotname_msgs::srv::SetIntakeMechanism>("set_intake_mechanism",
        std::bind(&SetIntakeMechanismServer::handle_service, this, _1, _2),
        qos_profile);
      _publisher = this->create_publisher<std_msgs::msg::Float32MultiArray>
        ("/set_mekanism", rclcpp::QoS(10));
    }

    void handle_service(const std::shared_ptr<robotname_msgs::srv::SetIntakeMechanism::Request> request, 
    const std::shared_ptr<robotname_msgs::srv::SetIntakeMechanism::Response> response)
    {
      std_msgs::msg::Float32MultiArray msg;
      msg.data.push_back(request->lift);
      msg.data.push_back(request->dribble);

      _publisher->publish(msg);
      response->status = true;
    }

  private:
    rclcpp::Service<robotname_msgs::srv::SetIntakeMechanism>::SharedPtr _service;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr _publisher;
};



int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<SetIntakeMechanismServer>node = std::make_shared<SetIntakeMechanismServer>();

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Set Speed Server Ready");

  rclcpp::spin(node);
  rclcpp::shutdown();
}