#include "robotname_autonomy/plugins/subscriber/grab_nearest_ball.hpp"

using namespace BT;

BT::PortsList GrabNearestBall::providedPorts()
  {
    return {InputPort<std::string>("color"),OutputPort<int>("id")};
  }

NodeStatus GrabNearestBall::onTick(const std::shared_ptr<robotname_msgs::msg::DetectionArray>& last_msg)
{
  if(last_msg) // empty if no new message received, since the last tick
  {
      //RCLCPP_INFO(logger(), "[%s] new message: %s", name().c_str(), last_msg->data.c_str());
        bool object_founded;
        std::vector<std::pair<double, int>> target_objects;
        std::string color;
        getInput("color",color);
        
        for(auto & object : last_msg->detections)
        {
          if (object.classname == color) {
            object_founded = true;
            double hypot_value = hypot(object.pose.pose.position.x, object.pose.pose.position.y);
            target_objects.push_back(std::make_pair(hypot_value, object.id));
          }

        }

        if(object_founded)
        {
          //RCLCPP_INFO(this->get_logger(), "Hit 2.5");
          if(!target_objects.empty())
          {
            std::sort(target_objects.begin(), target_objects.end());
            int result =target_objects.front().second;
            RCLCPP_INFO(node_->get_logger(), "[%s] Found Object with ID %d", name().c_str(), result);
            setOutput("id", result);
            return NodeStatus::SUCCESS;
            
          }
        }
        RCLCPP_INFO(node_->get_logger(), "[%s] Object Not Found", name().c_str());
  }
  else{
    RCLCPP_INFO(node_->get_logger(), "[%s] Empty msg topic", name().c_str());
  }
  return NodeStatus::FAILURE;
}             
