#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp" // Assuming you're subscribing to a string topic
#include "std_msgs/msg/int32.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include "rmw/qos_profiles.h"
#include "rclcpp/qos.hpp"

using std::placeholders::_1;

class BallCounterColor : public rclcpp::Node
{
  public:
    BallCounterColor()
    : Node("BallCounterColor")
    {
      publisher_ = this->create_publisher<std_msgs::msg::Int32>("ballcount",10);

        intake_sub.subscribe(this, "/intake/detectedcolor");
        intake2_sub.subscribe(this, "/intake2/detectedcolor");
            

        sync_policies = std::make_shared<message_filters::TimeSynchronizer<std_msgs::msg::String, std_msgs::msg::String>>(intake_sub, intake2_sub, 1);
        sync_policies->registerCallback(&BallCounterColor::topic_callback, this);

    }

  private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg1, const std_msgs::msg::String::SharedPtr msg2)
    {

        if(!first_msg)
        {
          last_msg1 = msg1;
          last_msg2 = msg2;
          first_msg = true;
          return;
        }
        if (last_msg1 && last_msg2)
        {
            std::string prev_intake_data = last_msg1->data;
            std::string prev_outtake_data = last_msg2->data;

            std::string intake_data = msg1->data;
            std::string outtake_data = msg2->data;

            if (intake_data != prev_intake_data)
            {
                if (intake_data == "purple")
                {
                    ballcount.data++;
                }
            }

            if (outtake_data != prev_outtake_data)
            {
                if (prev_outtake_data == "purple")
                {
                    ballcount.data--;
                }
            }

            if(ballcount.data < 0)
            {
              ballcount.data = 0;
            }
            publisher_->publish(ballcount);
            last_msg1 = msg1;
            last_msg2 = msg2;
        }
    }
    message_filters::Subscriber<std_msgs::msg::String> intake_sub;
    message_filters::Subscriber<std_msgs::msg::String> intake2_sub;


    std::shared_ptr<message_filters::TimeSynchronizer<std_msgs::msg::String, std_msgs::msg::String>> sync_policies;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    std_msgs::msg::String::SharedPtr last_msg1;
    std_msgs::msg::String::SharedPtr last_msg2;
    int first_msg;
    std_msgs::msg::Int32 ballcount;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BallCounterColor>());
  rclcpp::shutdown();
  return 0;
}
