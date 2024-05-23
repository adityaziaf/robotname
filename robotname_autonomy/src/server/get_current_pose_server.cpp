#include "rclcpp/rclcpp.hpp"
#include "robotname_msgs/srv/get_current_pose.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <memory>
#include "tf2/utils.h"
using namespace std::placeholders;

class GetCurrentPoseServer : public rclcpp::Node {
  public:
    GetCurrentPoseServer() : Node("GetCurrentPose")
    {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_default;

      _service = this->create_service<robotname_msgs::srv::GetCurrentPose>("get_current_pose_server",
        std::bind(&GetCurrentPoseServer::handle_service, this, _1, _2),
        qos_profile);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }
     ;

    void handle_service(const std::shared_ptr<robotname_msgs::srv::GetCurrentPose::Request> request, 
    const std::shared_ptr<robotname_msgs::srv::GetCurrentPose::Response> response)
    {
      
  
      try {
        geometry_msgs::msg::TransformStamped transform =
        tf_buffer_->lookupTransform(request->parent_frame_id,
                                    request->child_frame_id,
                                    tf2::TimePointZero);

        geometry_msgs::msg::PoseStamped in, out;   
        in.header.frame_id = request->child_frame_id;
        in.header.stamp = this->get_clock()->now();
        in.pose.position.x = 0;
        in.pose.position.y = 0;
        in.pose.position.z = 0;
        in.pose.orientation.x = 0;
        in.pose.orientation.y = 0;
        in.pose.orientation.x = 0;
        in.pose.orientation.w = 1;
        tf2::doTransform<geometry_msgs::msg::PoseStamped>(in, out, transform); 
        response->status = true;
        response->set__pose(out);
        response->set__x(out.pose.position.x);
        response->set__y(out.pose.position.y);
        response->set__theta(tf2::getYaw(out.pose.orientation));
        return ;

      } catch (const tf2::TransformException &ex) {
        RCLCPP_INFO(this->get_logger(), "Could not transform");
      }
        response->status = false;
    }
  private:
    rclcpp::Service<robotname_msgs::srv::GetCurrentPose>::SharedPtr _service;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};



int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<GetCurrentPoseServer>node = std::make_shared<GetCurrentPoseServer>();

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "GetCurrentPose Server Ready");

  rclcpp::spin(node);
  rclcpp::shutdown();
}