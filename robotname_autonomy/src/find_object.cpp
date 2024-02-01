// Vision related behaviors

#include "robotname_autonomy/find_object.hpp"

#include <unistd.h>

using std::placeholders::_1;

// LOOKFOROBJECT
// Looks for an object of a certain color, specified by a parameter
findObject::findObject(const std::string& name,
                       const BT::NodeConfiguration& config,
                       rclcpp::Node::SharedPtr node_ptr)
    : BT::StatefulActionNode(name, config), node_{node_ptr} {
  std::cout << "[" << this->name() << "] Initialized" << std::endl;
}

BT::NodeStatus findObject::onStart() {

  received_image_ = false;

  getInput<std::string>("objectname", objectname_);

  objects_subs_ =
      node_->create_subscription<robotname_msgs::msg::DetectionArray>(
          "/objects/transformed/tracked", rclcpp::QoS(10),
          std::bind(&findObject::detection_callback, this, _1));

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus findObject::onRunning() {
  // std::string target_color = "blue";
  std::cout << "[" << this->name() << "] Looking for " << objectname_
            << " object" << std::endl;

  // Wait to receive an image
  // TODO Add timeout?
  if (!received_image_) {
    // std::cout << "[" << this->name() << "] Waiting for image" << std::endl;
    return BT::NodeStatus::RUNNING;
  }

  geometry_msgs::msg::TransformStamped t;
  try {
    t = tf_buffer_->lookupTransform("map", "camera_link", tf2::TimePointZero);
  } catch (const tf2::TransformException& ex) {
    RCLCPP_INFO(node_->get_logger(), "Could not transform %s to %s: %s",
                s_frame.c_str(), t_frame.c_str(), ex.what());
    return BT::NodeStatus::RUNNING;
  }

  std::vector<std::pair<double, uint32_t>> target_objects;
  bool object_founded;

  for (auto object = latest_msg->detections.begin(); object != latest_msg->detections.end() ; object++) {

    if (object->classname == objectname_) {
      object_founded = true;
      // calculate euclidean distance
      double x_value = object->point.x - t.transform.translation.x;
      double y_value = object->point.y - t.transform.translation.y;
      double distance = std::sqrt(std::pow(x_value, 2) + std::pow(y_value, 2));
      target_objects.push_back(std::make_pair(distance,object->id));
      RCLCPP_INFO(node_->get_logger(), "distance:%f, object_id:%d", distance, object->id);
    }
  }

  if(object_founded)
  {
    std::sort(target_objects.begin(),target_objects.end());
    RCLCPP_INFO(node_->get_logger(), "classname:%s, object_id:%d, distance:%f", objectname_.c_str(), target_objects.begin()->second, target_objects.begin()->first);
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

void findObject::onHalted() { 
  received_image_ = false; 
}

BT::PortsList findObject::providedPorts() {
  return {BT::InputPort<std::string>("objectname")};
}

void findObject::detection_callback(
    const robotname_msgs::msg::DetectionArray::SharedPtr msg) {
  latest_msg = msg;
  received_image_ = true;
}