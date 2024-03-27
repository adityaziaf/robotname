#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "rmw/qos_profiles.h"
#include "rclcpp/qos.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

    class lidarfilter : public rclcpp::Node
    {
    public:


        lidarfilter() : Node("lidarfilter")
        {

            this->declare_parameter("roll_treshold", 0.0149066);
            this->declare_parameter("pitch_treshold", 0.0149066);
            
            this->get_parameter("roll_treshold", roll_treshold);
            this->get_parameter("pitch_treshold", pitch_treshold);

            laser_subs.subscribe(this, "/scan");
            odom_subs.subscribe(this,"/odom");

            my_sync_ = std::make_shared<approximate_synchronizer>(approximate_policy(1),laser_subs ,odom_subs);
            my_sync_->getPolicy()->setMaxIntervalDuration(rclcpp::Duration(1,0));
            my_sync_->registerCallback(&lidarfilter::filtercallback, this);

            laserpublisher = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan/filtered", 10);

        }

        ~lidarfilter()
        {
        }

    private:
        void filtercallback(const sensor_msgs::msg::LaserScan::SharedPtr lasermsg, const nav_msgs::msg::Odometry::SharedPtr odommsg)
        {
            tf2::Quaternion quaternion;
            tf2::fromMsg(odommsg->pose.pose.orientation, quaternion);
        
            tf2::Matrix3x3 rotation_matrix(quaternion);
            double roll, pitch, yaw;
            rotation_matrix.getRPY(roll, pitch, yaw);
            
            

            if(-roll_treshold <= roll && roll <= roll_treshold)
            {
                if(-pitch_treshold <= pitch && pitch <= pitch_treshold)
                {
                    std::cout << "Roll: " << roll << std::endl;
                    std::cout << "Pitch: " << pitch << std::endl;
                    std::cout << "Yaw: " << yaw << std::endl;
                    laserpublisher->publish(*lasermsg);
                }
            }
        }
        double pitch_treshold, roll_treshold;
        message_filters::Subscriber<sensor_msgs::msg::LaserScan> laser_subs;
        message_filters::Subscriber<nav_msgs::msg::Odometry> odom_subs;

        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laserpublisher;
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::LaserScan, nav_msgs::msg::Odometry> approximate_policy;
        typedef message_filters::Synchronizer<approximate_policy> approximate_synchronizer;
        std::shared_ptr<approximate_synchronizer> my_sync_;

    };

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lidarfilter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
