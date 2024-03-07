// Copyright 2020 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "robotname_msgs/msg/detection_array.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class ClickedPointToPose : public rclcpp::Node
{
public:
  ClickedPointToPose(const std::string & name)
  : Node(name)
  {
    sub_ = create_subscription<robotname_msgs::msg::DetectionArray>(
      "/objects/transformed", 10, std::bind(&ClickedPointToPose::callback_clicked, this, _1));
    timer_ = create_wall_timer((std::chrono::milliseconds) 500,
                                      std::bind(&ClickedPointToPose::callback_timer, this));
    pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);
  }
  robotname_msgs::msg::DetectionArray::SharedPtr latest_msg;
private:
  void callback_clicked(const robotname_msgs::msg::DetectionArray::SharedPtr msg) const
  {
    if(!msg->detections.empty())
    {
        for(auto &object : msg->detections)
        {
          for(auto &object : msg->detections)
        {
            if(object.classname == "redball")
            {
                geometry_msgs::msg::PoseStamped pose;
                pose.header = object.pose.header;
                pose.pose.position = object.pose.pose.position;
                pose.pose.orientation.x = 0;
                pose.pose.orientation.y = 0;
                pose.pose.orientation.z = 0;
                pose.pose.orientation.w = 1;

                pub_->publish(pose);
                return;
            }
    
    }
    }
    std::this_thread::sleep_for(500ms);
    }
  }

  void callback_timer() 
  {
    
  }
  rclcpp::Subscription<robotname_msgs::msg::DetectionArray>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto clicked_point_to_pose_node = std::make_shared<ClickedPointToPose>("clicked_point_to_pose");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(clicked_point_to_pose_node);
 
  executor.spin();

  rclcpp::shutdown();

  return 0;
}