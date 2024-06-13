#include "rclcpp/rclcpp.hpp"
#include "robotname_msgs/srv/find_nearest_ball.hpp"
#include "robotname_msgs/msg/detection_array.hpp"

#include <memory>

using namespace std::placeholders;

class FindNearestBall : public rclcpp::Node {
  public:
    FindNearestBall() : Node("FindNearestBall")
    {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_default;

      _service = this->create_service<robotname_msgs::srv::FindNearestBall>("get_nearest_ball",
        std::bind(&FindNearestBall::handle_service, this, _1, _2),
        qos_profile);
      _subscriber = this->create_subscription<robotname_msgs::msg::DetectionArray>
        ("/camera1/objects/tracked", rclcpp::QoS(10),
            std::bind(&FindNearestBall::detection_callback, this, _1));
    }
     ;

    void handle_service(const std::shared_ptr<robotname_msgs::srv::FindNearestBall::Request> request, 
    const std::shared_ptr<robotname_msgs::srv::FindNearestBall::Response> response)
    {
      
  
      if(_last_msg)
      {
        bool object_founded;
        std::vector<std::pair<double, int>> target_objects;

        for(auto & object : _last_msg->detections)
        {
          if (object.classname == request->color) {
            object_founded = true;
            //RCLCPP_INFO(this->get_logger(), "Hit 0");
            double hypot_value = hypot(object.pose.pose.position.x, object.pose.pose.position.y);
            target_objects.push_back(std::make_pair(hypot_value, object.id));
            //RCLCPP_INFO(this->get_logger(), "Hit 1");
          }

        }

        if(object_founded)
        {
          //RCLCPP_INFO(this->get_logger(), "Hit 2.5");
          if(!target_objects.empty())
          {
            std::sort(target_objects.begin(), target_objects.end());RCLCPP_INFO(this->get_logger(), "Hit 1");
            response->id=target_objects.front().second;
            response->status=true;
            return;
          }
        }
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "Empty Topic Value");
      }
      response->status=false;
    }

    void detection_callback(const robotname_msgs::msg::DetectionArray::SharedPtr msg)
    {
      _last_msg = msg;
    }

  private:
    rclcpp::Service<robotname_msgs::srv::FindNearestBall>::SharedPtr _service;
    rclcpp::Subscription<robotname_msgs::msg::DetectionArray>::SharedPtr _subscriber;
    robotname_msgs::msg::DetectionArray::SharedPtr _last_msg;

};



int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<FindNearestBall>node = std::make_shared<FindNearestBall>();

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Find Nearest Ball Server Ready");

  rclcpp::spin(node);
  rclcpp::shutdown();
}