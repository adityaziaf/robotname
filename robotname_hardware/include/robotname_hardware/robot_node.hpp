#ifndef __ROBOT_ITSROBOCON_ROBOT_NODE__
#define __ROBOT_ITSROBOCON_ROBOT_NODE__

#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <vector>

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "robot_itsrobocon_msgs/msg/polar_vector_array.hpp"
#include "robot_itsrobocon_msgs/msg/odometry.hpp"
#include "robot_itsrobocon_msgs/msg/swerve.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "sensor_msgs/msg/imu.hpp"
// #include "sensor_msgs/msg/joy.hpp"

class robotNode : public rclcpp::Node { 
public:
    robotNode();

private:
    /* Method */
    int udpLoop();
    // int swerveSub(const robot_itsrobocon_msgs::msg::PolarVectorArray &msg);
    int flagSub(const std_msgs::msg::UInt32 &msg);
    int speedSub(const geometry_msgs::msg::Twist &msg);
    // void odomResetSub(const std_msgs::msg::Bool &msg);
    void odomResetSub(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                                   std::shared_ptr<std_srvs::srv::Empty::Response> res);
    void robot_status_callback(const std_msgs::msg::Bool &msg);
    // void gripper_callback(const std_msgs::msg::Float32MultiArray &msg);

    /* Subscriptor */
    // rclcpp::Subscription<robot_itsrobocon_msgs::msg::PolarVectorArray>::SharedPtr cmd_swerve_vector_sub;
    rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr cmd_flag_sub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_base_vel;
    // rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr odom_reset_sub;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr odom_reset_service;

    //Subscription topic status robot, apakah robot dalam kondisi run atau idle
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr run_status_sub;

    // rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
    // rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr seedling_sub;

    /* Publisher */
    // rclcpp::Publisher<robot_base_ros2::msg::FromStm>::SharedPtr publisher;
    // rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr robot_g_position_publish;
    // rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr g_position_pub;
    // rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr body_speed_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
    // rclcpp::Publisher<robot_itsrobocon_msgs::msg::Swerve>::SharedPtr act_swerve_pub;
    // rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr act_swerve_current_pub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr v_bat_pub;

    /* Timer */
    rclcpp::TimerBase::SharedPtr udp_timer;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};



#endif /*__ROBOT_ITSROBOCON_ROBOT_NODE__*/