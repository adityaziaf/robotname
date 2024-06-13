#include "rclcpp/rclcpp.hpp"
#include "robotname_msgs/srv/ball_available.hpp"
#include "robotname_msgs/msg/detection_array.hpp"

#include <memory>

using namespace std::placeholders;

class BallAvailable : public rclcpp::Node {
  public:
    BallAvailable() : Node("BallAvailable")
    {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_default;

      _service = this->create_service<robotname_msgs::srv::BallAvailable>("ball_available",
        std::bind(&BallAvailable::handle_service, this, _1, _2),
        qos_profile);
      _subscriber = this->create_subscription<robotname_msgs::msg::DetectionArray>
        ("/camera1/objects/tracked", rclcpp::QoS(1),
            std::bind(&BallAvailable::detection_callback, this, _1));
    }

    void handle_service(const std::shared_ptr<robotname_msgs::srv::BallAvailable::Request> request, 
    const std::shared_ptr<robotname_msgs::srv::BallAvailable::Response> response)
    {
      
      if(!_last_msg->detections.empty())
      {
        for(auto & object : _last_msg->detections)
        {
          if (object.classname == request->color) {
            if(object.id == request->id)
            {
              response->status = true;
              response->set__pose(object.pose);
              return;
            }
          }
        }
      }
      response->status = false;
    }

    void detection_callback(const robotname_msgs::msg::DetectionArray::SharedPtr msg)
    {
      _last_msg = msg;
    }

  private:
    rclcpp::Service<robotname_msgs::srv::BallAvailable>::SharedPtr _service;
    rclcpp::Subscription<robotname_msgs::msg::DetectionArray>::SharedPtr _subscriber;
    robotname_msgs::msg::DetectionArray::SharedPtr _last_msg;

};



int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<BallAvailable>node = std::make_shared<BallAvailable>();

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ball Available Server Ready");

  rclcpp::spin(node);
  rclcpp::shutdown();
}