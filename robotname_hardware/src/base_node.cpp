/**
 ******************************************************************************
  * File Name          : base_node.cpp
  * Description        : File ini adalah file penggerak base (swerve)
  * Author             : ITS Robocon 
  ******************************************************************************
  */
#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "robot_itsrobocon_msgs/msg/polar_vector_array.hpp"
#include "robot_itsrobocon_msgs/msg/path2_d.hpp"
#include "robot_itsrobocon_msgs/msg/swerve.hpp"
#include "robotname_hardware/xytheta.h"

class baseNode : public rclcpp::Node {
    public:
        //Subscription topic status robot, apakah robot dalam kondisi run atau idle
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr run_status_sub;

        //Subscription topic command kecepatan robot
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr base_twist_sub;

        //Subscription topic command waypoint robot
        rclcpp::Subscription<robot_itsrobocon_msgs::msg::Path2D>::SharedPtr base_path_sub;

        //Subscription topic kecepatan aktual swerve
        rclcpp::Subscription<robot_itsrobocon_msgs::msg::Swerve>::SharedPtr swerve_sub;

        //Subscription topic robot_position
        rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr global_pos_sub;

        //Publisher swerve vel command
        rclcpp::Publisher<robot_itsrobocon_msgs::msg::PolarVectorArray>::SharedPtr cmd_swerve_vector_pub;

        //Publisher kondisi path
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr path_flag_pub;

        baseNode() : Node("base_node") {
            
            base_twist_sub = this->create_subscription<geometry_msgs::msg::Twist> (
                        "base_vel", 10, std::bind(&baseNode::base_vel_callback, this, std::placeholders::_1));

            base_path_sub = this->create_subscription<robot_itsrobocon_msgs::msg::Path2D> (
                        "base_path", 2, std::bind(&baseNode::base_path_callback, this, std::placeholders::_1));

            run_status_sub = this->create_subscription<std_msgs::msg::Bool> (
                        "run_status", 5, std::bind(&baseNode::robot_status_callback, this, std::placeholders::_1));

            swerve_sub     = this->create_subscription<robot_itsrobocon_msgs::msg::Swerve> (
                                "swerve", 5, std::bind(&baseNode::swerve_callback, this, std::placeholders::_1));

            auto qos = rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, 
                                                            rmw_qos_profile_sensor_data.history), 
                                rmw_qos_profile_sensor_data);
                                
            global_pos_sub = this->create_subscription<geometry_msgs::msg::Pose2D>(
                            "global_pos", qos, std::bind(&baseNode::global_pos_callback, this, std::placeholders::_1));

            cmd_swerve_vector_pub = this->create_publisher<robot_itsrobocon_msgs::msg::PolarVectorArray>("cmd_swerve_vel", 10);

            path_flag_pub = this->create_publisher<std_msgs::msg::Bool>("path_flag", 10);

            RCLCPP_INFO(this->get_logger(), "Base Node started!");
        }   

    private:
        void base_vel_callback(const geometry_msgs::msg::Twist &msg) {
            //Program menggerakkan base dengan kecepatan tertentu, sesuai data msg
        }

        void base_path_callback(const robot_itsrobocon_msgs::msg::Path2D &msg) {
            //Program menggerakkan base melalui waypoint yang diinstruksikan dalam msg.path dengan kecepatan msg.speed
        }

        void robot_status_callback(const std_msgs::msg::Bool &msg) {
            //Program force stop saat msg=false
        }

        void swerve_callback(const robot_itsrobocon_msgs::msg::Swerve &msg) {
            
        }

        void global_pos_callback(const geometry_msgs::msg::Pose2D &msg)
        {

        }

};

/**
 * @brief Fungsi ini berguna untuk..... ya kalian tau lah
 * @param argc argument count
 * @param argv argument c_str array
 * @retval null
*/

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<baseNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
