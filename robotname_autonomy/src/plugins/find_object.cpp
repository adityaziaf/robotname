// Vision related behaviors

#include "robotname_autonomy/plugins/find_object.hpp"

#include <unistd.h>

using std::placeholders::_1;

// LOOKFOROBJECT
// Looks for an object of a certain color, specified by a parameter
findObject::findObject(const std::string& name,
                       const BT::NodeConfiguration& config,
                       rclcpp::Node::SharedPtr node_ptr)
    : BT::StatefulActionNode(name, config), node_{node_ptr} {
  RCLCPP_INFO(node_->get_logger(),"[%s] Initialized", this->name().c_str());
}

BT::NodeStatus findObject::onStart() {
  received_image_ = false;

  getInput<std::string>("object", objectname_);

  objects_subs_ =
      node_->create_subscription<robotname_msgs::msg::DetectionArray>(
          "/objects/transformed/tracked", rclcpp::QoS(10),
          std::bind(&findObject::detection_callback, this, _1));

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  t_frame_ = "map";
  s_frame_ = "base_link";

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus findObject::onRunning() {
  // std::string target_color = "blue";
  RCLCPP_INFO(node_->get_logger(),"[%s] Looking for %s object", this->name().c_str(), objectname_.c_str());

  // Wait to receive an image
  // TODO Add timeout?
  if (!received_image_) {
    // std::cout << "[" << this->name() << "] Waiting for image" << std::endl;
    return BT::NodeStatus::RUNNING;
  }

  geometry_msgs::msg::TransformStamped t;
  try {
    t = tf_buffer_->lookupTransform(t_frame_, s_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException& ex) {
    RCLCPP_INFO(node_->get_logger(),"[%s] Could not transform %s to %s: %s", this->name().c_str(),
           s_frame_.c_str(), t_frame_.c_str(), ex.what());
    return BT::NodeStatus::RUNNING;
  }

  std::vector<std::pair<double, int>> target_objects;
  bool object_founded;

  if (!latest_msg->detections.empty()) {
    for (auto object = latest_msg->detections.begin();
         object != latest_msg->detections.end(); object++) {
      if (object->classname == objectname_) {
        object_founded = true;
        // calculate euclidean distance
        double x_value =
            object->pose.pose.position.x - t.transform.translation.x;
        double y_value =
            object->pose.pose.position.y - t.transform.translation.y;
        double distance =
            std::sqrt(std::pow(x_value, 2) + std::pow(y_value, 2));
        target_objects.push_back(std::make_pair(distance, object->id));
         RCLCPP_INFO(node_->get_logger(),"[%s] distance:%f, object_id:%d", this->name().c_str(), distance,
               object->id);
      }
    }

    if (object_founded) {
      std::sort(target_objects.begin(), target_objects.end());
       RCLCPP_INFO(node_->get_logger(),"[%s] classname:%s, object_id:%d, distance:%f", this->name().c_str(),
             objectname_.c_str(), target_objects.begin()->second,
             target_objects.begin()->first);
      setOutput<int>("id", target_objects.begin()->second);
      return BT::NodeStatus::SUCCESS;
    }
  }
  return BT::NodeStatus::FAILURE;
}

void findObject::onHalted() { received_image_ = false; }

BT::PortsList findObject::providedPorts() {
  return {BT::InputPort<std::string>("object"), BT::OutputPort<int>("id")};
}

void findObject::detection_callback(
    const robotname_msgs::msg::DetectionArray::SharedPtr msg) {
  latest_msg = msg;
  received_image_ = true;
}