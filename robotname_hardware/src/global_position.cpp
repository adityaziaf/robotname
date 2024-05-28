#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"

using namespace std::chrono_literals;

class BaseLinkPublisher : public rclcpp::Node {
public:
    BaseLinkPublisher() : Node("base_link_publisher") {
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/base_link", 10);

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);

        // Using createTimer from tf2_ros to avoid issues with the buffer timing out
        tf2_ros::CreateTimerROS create_timer_interface(this->get_node_base_interface(), this->get_node_timers_interface());
        tf_buffer_->setCreateTimerInterface(&create_timer_interface);

        timer_ = this->create_wall_timer(500ms, std::bind(&BaseLinkPublisher::publishBaseLink, this));
    }

private:
    void publishBaseLink() {
        try {
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);

            auto message = std::make_unique<geometry_msgs::msg::PoseStamped>();
            message->header.stamp = transform.header.stamp;  // Use the transform's timestamp
            message->header.frame_id = transform.header.frame_id;
            message->pose.position.x = transform.transform.translation.x;
            message->pose.position.y = transform.transform.translation.y;
            message->pose.position.z = transform.transform.translation.z;
            message->pose.orientation = transform.transform.rotation;

            publisher_->publish(std::move(message));
        } catch (const tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get transform from map to base_link: %s", ex.what());
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BaseLinkPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
