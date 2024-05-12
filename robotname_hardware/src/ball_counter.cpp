#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp" // Assuming you're subscribing to a string topic
#include "std_msgs/msg/int32.hpp"

using std::placeholders::_1;

class BallCounter : public rclcpp::Node
{
  public:
    BallCounter()
    : Node("BallCounter")
    {
      subscription_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
      "proximity_array", 10, std::bind(&BallCounter::topic_callback, this, _1));
      publisher_ = this->create_publisher<std_msgs::msg::Int32>("ballcount",10);
    }

  private:
    void topic_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
    {

        if(!first_msg)
        {
          last_msg = msg;
          first_msg = true;
          return;
        }
        if (msg)
        {
            uint8_t prev_intake_data = last_msg->data.at(1);
            uint8_t prev_outtake_data = last_msg->data.back();

            uint8_t intake_data = msg->data.at(1);
            uint8_t outtake_data = msg->data.back();

            if (intake_data != prev_intake_data)
            {
                if (intake_data == 1)
                {
                    ballcount.data++;
                }
            }

            if (outtake_data != prev_outtake_data)
            {
                if (outtake_data == 0)
                {
                    ballcount.data--;
                }
            }

            if(ballcount.data < 0)
            {
              ballcount.data = 0;
            }
            publisher_->publish(ballcount);
            last_msg = msg;
        }
    }
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr subscription_;
    std_msgs::msg::UInt8MultiArray::SharedPtr last_msg;
    int first_msg;
    std_msgs::msg::Int32 ballcount;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BallCounter>());
  rclcpp::shutdown();
  return 0;
}
