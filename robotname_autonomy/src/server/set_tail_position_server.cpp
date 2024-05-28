#include "rclcpp/rclcpp.hpp"
#include "robotname_msgs/srv/set_tail_position.hpp"
#include "std_msgs/msg/float32.hpp"

#include <memory>

using namespace std::placeholders;

class SetTailPositionServer : public rclcpp::Node {
  public:
    SetTailPositionServer() : Node("SetTailPositionServer")
    {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_services_default;

      _service = this->create_service<robotname_msgs::srv::SetTailPosition>("set_tail_position",
        std::bind(&SetTailPositionServer::handle_service, this, _1, _2),
        qos_profile);
      _publisher = this->create_publisher<std_msgs::msg::Float32>
        ("/set_tail", rclcpp::QoS(10));
    }

    void handle_service(const std::shared_ptr<robotname_msgs::srv::SetTailPosition::Request> request, 
    const std::shared_ptr<robotname_msgs::srv::SetTailPosition::Response> response)
    {
      std_msgs::msg::Float32 msg;
      msg.data=request->angle;

      _publisher->publish(msg);
      response->status = true;
    }

  private:
    rclcpp::Service<robotname_msgs::srv::SetTailPosition>::SharedPtr _service;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _publisher;
};



int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<SetTailPositionServer>node = std::make_shared<SetTailPositionServer>();

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Set Tail Position Server Ready");

  rclcpp::spin(node);
  rclcpp::shutdown();
}