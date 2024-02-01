/**
 ******************************************************************************
  * File Name          : main_node.cpp
  * Description        : File ini adalah file utama untuk main_node
  *                      berisi file manager, action manager dan sequence
  *                      manager
  ******************************************************************************
  */
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <vector>
#include <string>

#include "robot_itsrobocon_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "std_srvs/srv/empty.hpp"

class positionNode : public rclcpp::Node {
public:

    rclcpp::Subscription<robot_itsrobocon_msgs::msg::Odometry>::SharedPtr odometry_sub;
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr global_pos_pub;

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr global_pos_reset_service;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr odom_reset_client;

    positionNode() : Node("position_node") {
        
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, rmw_qos_profile_sensor_data.history), rmw_qos_profile_sensor_data);

        odometry_sub = this->create_subscription<robot_itsrobocon_msgs::msg::Odometry> (
                       "odometry", qos, std::bind(&positionNode::odom_callback, this, std::placeholders::_1));
        global_pos_pub = this->create_publisher<geometry_msgs::msg::Pose2D>("global_pos", qos);

        odom_reset_client = this->create_client<std_srvs::srv::Empty>("odometry/reset");
        global_pos_reset_service = this->create_service<std_srvs::srv::Empty>(
                                "global_pos/reset", std::bind(&positionNode::global_pos_reset, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Position Node started!");
    }
private:
    bool global_pos_reset_status = true;

    void odom_callback(const robot_itsrobocon_msgs::msg::Odometry &msg) {
        auto temp = geometry_msgs::msg::Pose2D();

        temp.x = msg.pose.x;
        temp.y = msg.pose.y;
        temp.theta = msg.pose.theta;

        global_pos_pub->publish(temp);

        if(global_pos_reset_status == true) {
            auto req = std::make_shared<std_srvs::srv::Empty::Request>();
            auto resp = odom_reset_client->async_send_request(req, [this](rclcpp::Client<std_srvs::srv::Empty>::SharedFuture future) {
                global_pos_reset_status = false;
                future.get();
            });
        }
    }


    void global_pos_reset(const std::shared_ptr<std_srvs::srv::Empty::Request> req, 
                                std::shared_ptr<std_srvs::srv::Empty::Response> resp) {
        global_pos_reset_status = true;
        req.get();
        resp.get();
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
  auto node = std::make_shared<positionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}