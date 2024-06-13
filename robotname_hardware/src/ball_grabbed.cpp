#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"


#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

using std::placeholders::_1;

class BallGrabbed : public rclcpp::Node
{
  public:
    BallGrabbed()
    : Node("BallGrabbed")
    {

      rclcpp::QoS qos(rclcpp::KeepLast(1), rmw_qos_profile_default);

      prox_sub.subscribe(this, "/proximity_array", qos.get_rmw_qos_profile());
      color_sub.subscribe(this, "/camera1/aligned_depth_to_color/image_raw", qos.get_rmw_qos_profile());

      sync_policies = std::make_shared<message_filters::TimeSynchronizer<std_msgs::msg::UInt8MultiArray,std_msgs::msg::String>>(prox_sub, color_sub, 1);
      sync_policies->registerCallback(&BallGrabbed::topic_callback, this);

      publisher_ = this->create_publisher<std_msgs::msg::Bool>("ballgrabbed",10);
    }

  private:
    void topic_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr prox_msg, const std_msgs::msg::String::SharedPtr color_msg)
    {
        if(color_msg->data)
    }
    message_filters::Subscriber<std_msgs::msg::UInt8MultiArray> prox_sub;
    message_filters::Subscriber<std_msgs::msg::String> color_sub;


    std::shared_ptr<message_filters::TimeSynchronizer<std_msgs::msg::UInt8MultiArray, std_msgs::msg::String>> sync_policies;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BallGrabbed>());
  rclcpp::shutdown();
  return 0;
}
