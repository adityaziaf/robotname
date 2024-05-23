#include "rclcpp/rclcpp.hpp"
#include "robotname_msgs/srv/ball_available.hpp"
#include "robotname_msgs/srv/ball_grabbed.hpp"
#include "robotname_msgs/msg/detection_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

#include <memory>

using namespace std::placeholders;

class BallGrabbedServer : public rclcpp::Node {
  public:
    BallGrabbedServer() : Node("BallGrabbedServer")
    {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_default;

      _service = this->create_service<robotname_msgs::srv::BallGrabbed>("ball_grabbed",
        std::bind(&BallGrabbedServer::handle_service, this, _1, _2),
        qos_profile);
      _subscriber = this->create_subscription<robotname_msgs::msg::DetectionArray>
        ("/camera/objects/tracked", rclcpp::QoS(1),
            std::bind(&BallGrabbedServer::detection_callback, this, _1));

      _intake_prox_sub = this->create_subscription<std_msgs::msg::UInt8MultiArray>("/proximity_array", 1, std::bind(&BallGrabbedServer::handle_prox_subscription, this,_1));
      _intake_color_sub = this->create_subscription<std_msgs::msg::String>("/intake/detectedcolor", 1, std::bind(&BallGrabbedServer::handle_color_subscription, this,_1));
     
    }

    void handle_service(const std::shared_ptr<robotname_msgs::srv::BallGrabbed::Request> request, 
    const std::shared_ptr<robotname_msgs::srv::BallGrabbed::Response> response)
    {
      (void)request;

      if(last_color_msg && last_prox_msg)
      {
        if(last_prox_msg->data.front() == 1)
        { 
          response->color = last_color_msg->data;  
          response->status = true;
        }       
      }
    }

    void detection_callback(const robotname_msgs::msg::DetectionArray::SharedPtr msg)
    {
      _last_msg = msg;
    }

    void handle_color_subscription(const std_msgs::msg::String::SharedPtr msg)
    {
      last_color_msg = msg;
    }

    void handle_prox_subscription(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
    {
      last_prox_msg = msg;
    }

  private:
    rclcpp::Service<robotname_msgs::srv::BallGrabbed>::SharedPtr _service;
    rclcpp::Subscription<robotname_msgs::msg::DetectionArray>::SharedPtr _subscriber;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr _intake_prox_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _intake_color_sub;
    std_msgs::msg::String::SharedPtr last_color_msg;
    std_msgs::msg::UInt8MultiArray::SharedPtr last_prox_msg;
    robotname_msgs::msg::DetectionArray::SharedPtr _last_msg;

};



int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<BallGrabbedServer>node = std::make_shared<BallGrabbedServer>();

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ball Available Server Ready");

  rclcpp::spin(node);
  rclcpp::shutdown();
}