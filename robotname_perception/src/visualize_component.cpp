#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "robotname_msgs/msg/detection.hpp"
#include "robotname_msgs/msg/detection_array.hpp"
#include "robotname_perception/visibility_control.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace robotname_perception {

using std::placeholders::_1;

class visualizeComponent : public rclcpp::Node {
 public:
  COMPOSITION_PUBLIC

  explicit visualizeComponent(const rclcpp::NodeOptions &options)
      : Node("visualizecomponent", options) {

    viz_subs_ =
        this->create_subscription<robotname_msgs::msg::DetectionArray>(
            "/omni/objects/tracked", rclcpp::QoS(10),
            std::bind(&visualizeComponent::callback, this, _1));
    viz_pubs_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/omni/objects/visualize", rclcpp::QoS(10));

  }

  ~visualizeComponent() {}

 private:
  void callback(const robotname_msgs::msg::DetectionArray::SharedPtr msg) {
    visualization_msgs::msg::MarkerArray markarray;
    for(auto object = msg->detections.begin() ; object != msg->detections.end();object++)
    {
        visualization_msgs::msg::Marker marker;
        marker.ns = object->classname;
        marker.id = object->id;
        // marker.text = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.set__header(object->pose.header);
        marker.set__pose(object->pose.pose);
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.a = 1.0;

        if(object->classname == "redball")
        {
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
        }
        else if(object->classname == "blueball")
        {
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
        }
        else if(object->classname == "purpleball")
        {
            marker.color.r = 0.8;
            marker.color.g = 0.1;
            marker.color.b = 0.8;
        }

        marker.lifetime = rclcpp::Duration::from_seconds(1);

        markarray.markers.push_back(marker);
    }
    viz_pubs_->publish(markarray);
  }
  rclcpp::Subscription<robotname_msgs::msg::DetectionArray>::SharedPtr
      viz_subs_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      viz_pubs_;
};
}  // namespace robotname_perception

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be
// discoverable when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(robotname_perception::visualizeComponent)
