#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "rmw/qos_profiles.h"
#include "rclcpp/qos.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/utils.h"
using std::placeholders::_1;
    class lidarreference : public rclcpp::Node
    {
    public:
        lidarreference() : Node("lidarreference")
        {
            laser_subs = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&lidarreference::lidarcallback, this, _1));
            // laser_subs =this->create_subscription<sensor_msgs::msg::LaserScan>(
            // "scan", rclcpp::QoS(10),std::bind(&lidarreference::lidarcallback, this, _1));            
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            laserpublisher = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan/transform", 10);
        }

        ~lidarreference()
        {
        }

    private:
        void lidarcallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
        geometry_msgs::msg::TransformStamped transform;
        try
        {
            transform = tf_buffer_->lookupTransform(
                "base_link", scan_msg->header.frame_id,
                scan_msg->header.stamp, tf2::durationFromSec(0.5));
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform %s to base_link: %s",
                        scan_msg->header.frame_id.c_str(), ex.what());
            return;
        }

        sensor_msgs::msg::LaserScan transformed_scan = *scan_msg;
        transformed_scan.header.frame_id = "base_link";

        std::vector<float> transformed_ranges(transformed_scan.ranges.size(), std::numeric_limits<float>::infinity());


        // Recompute angle_min and angle_max based on the new range data
        double new_angle_min = transformed_scan.angle_min + tf2::getYaw(transform.transform.rotation);
        double new_angle_max = transformed_scan.angle_min + tf2::getYaw(transform.transform.rotation) + (transformed_scan.ranges.size()) * transformed_scan.angle_increment;

        transformed_scan.angle_min = new_angle_min;
        transformed_scan.angle_max = new_angle_max;

        for (size_t i = 0; i < scan_msg->ranges.size(); ++i)
        {
            if (std::isfinite(scan_msg->ranges[i]))
            {
                double angle = scan_msg->angle_min + i * scan_msg->angle_increment;
                geometry_msgs::msg::PointStamped laser_point;
                laser_point.header.frame_id = scan_msg->header.frame_id;
                laser_point.point.x = scan_msg->ranges[i] * cos(angle);
                laser_point.point.y = scan_msg->ranges[i] * sin(angle);
                laser_point.point.z = 0.0;

                geometry_msgs::msg::PointStamped base_point;
                tf2::doTransform(laser_point, base_point, transform);

                double range = sqrt(pow(base_point.point.x, 2) + pow(base_point.point.y, 2));
                double transformed_angle = atan2(base_point.point.y, base_point.point.x);

                int transformed_index = static_cast<int>((transformed_angle - transformed_scan.angle_min) / transformed_scan.angle_increment);
                if (transformed_index >= 0 && transformed_index < static_cast<int>(transformed_scan.ranges.size()))
                {
                    transformed_ranges[transformed_index] = range;
                }
            }
        }

        transformed_scan.ranges = transformed_ranges;

        laserpublisher->publish(transformed_scan);
    }
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subs;
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laserpublisher;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    };

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lidarreference>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
