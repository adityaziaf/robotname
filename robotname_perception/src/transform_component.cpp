#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robotname_msgs/msg/detection.hpp"
#include "robotname_msgs/msg/detection_array.hpp"
#include "robotname_perception/visibility_control.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace robotname_perception {

using std::placeholders::_1;

class transformComponent : public rclcpp::Node {
 public:
  COMPOSITION_PUBLIC

  explicit transformComponent(const rclcpp::NodeOptions &options)
      : Node("transformcomponent", options) {
    this->declare_parameter("source_frame", "camera_color_optical_frame");
    this->declare_parameter("target_frame", "map");

    this->get_parameter("source_frame", s_frame);
    this->get_parameter("target_frame", t_frame);

    detection_subs_ =
        this->create_subscription<robotname_msgs::msg::DetectionArray>(
            "/objects/raw", rclcpp::QoS(10),
            std::bind(&transformComponent::callback, this, _1));
    detection_pubs_ =
        this->create_publisher<robotname_msgs::msg::DetectionArray>(
            "/objects/transformed", rclcpp::QoS(10));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  ~transformComponent() {}

 private:
  void callback(const robotname_msgs::msg::DetectionArray::SharedPtr msg) {
    geometry_msgs::msg::TransformStamped t;
    try {
      t = tf_buffer_->lookupTransform(t_frame, s_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",
                  s_frame.c_str(), t_frame.c_str(), ex.what());
      return;
    }
    for (auto object = msg->detections.begin(); object != msg->detections.end();
         object++) {
      object->header.frame_id = t_frame;
      object->header.stamp = this->get_clock()->now();
      object->point.x += t.transform.translation.x;
      object->point.y += t.transform.translation.y;
      object->point.z += t.transform.translation.z;
    }
    detection_pubs_->publish(*msg);
  }
  rclcpp::Subscription<robotname_msgs::msg::DetectionArray>::SharedPtr
      detection_subs_;
  rclcpp::Publisher<robotname_msgs::msg::DetectionArray>::SharedPtr
      detection_pubs_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string t_frame, s_frame;
};
}  // namespace robotname_perception

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be
// discoverable when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(robotname_perception::transformComponent)
