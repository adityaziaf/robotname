#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include <std_msgs/msg/float32_multi_array.hpp>
#include "robotname_autonomy/plugins/set_intake_mechanism.hpp"

PortsList SetIntakeMechanism::providedPorts()
  {
    return { InputPort<float>("dribble"),
             InputPort<float>("lift") 
           };
  }
  
bool SetIntakeMechanism::setMessage(std_msgs::msg::Float32MultiArray &msg)
{
    float dribble, lift;
    getInput("dribble", dribble);
    getInput("lift", lift);

    msg.data.push_back(lift);
    msg.data.push_back(dribble);
    return true;
}


