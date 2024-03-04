#include "robotname_hardware/robot_node.hpp"

#include <bitset>
#include <vector>

#include "robotname_hardware/udp.hpp"

#define IP_SERVER "192.168.1.111"
#define IP_CLIENT "192.168.1.134"
#define PORT 12345

UDP n_udp(IP_SERVER, PORT);

// using namespace std::chrono_literals;

#define NUM_SWERVE 3

// struct send_to_robot{
//     uint32_t timestamp;

//     struct {
//         float r;
//         float theta;
//     } swerve_speed[NUM_SWERVE];

//     float lift_speed;
//     float feeder_speed;
//     float shooter_speed;

//     float roller_speed_upper;
//     float roller_speed_lower;

//     struct {
//         float x;
//         float y;
//         float theta;
//     } body_speed;

//     union {
//         struct {
//             uint8_t reset_odom  : 1;
//         };
//         uint32_t flag;
//     };
// };

// struct recv_from_robot{
//     uint32_t timestamp;

//     struct {
//         int32_t x;
//         int32_t y;
//         float theta;
//     } g_position;

//     struct {
//         float x;
//         float y;
//         float theta;
//     } body_speed;

// 	//Swerve
//     struct {
//         float r;
//         float theta;
//     } swerve[NUM_SWERVE];
//     float swerve_drive_current[NUM_SWERVE];

//     int16_t lifter_pos;
//     int16_t feeder_pos;
//     int16_t shooter_pos;

//     union {
//     	struct {
//     		uint8_t lifter_sensor	: 1;

//         	uint8_t ring_sensor		: 1;
//     	};
//     	uint16_t sensor_status;
//     };

//     float v_bat;
//     uint32_t global_flag;
// };

struct send_to_robot {
  uint32_t timestamp;

  struct {
    float x;
    float y;
    float theta;
  } body_speed;

  union {
    struct {
      uint8_t reset_odom : 1;
      // uint8_t gripper_claw : 1;
    };
    uint32_t flag;
  };
};

struct recv_from_robot {
  uint32_t timestamp;

  struct {
    float x;
    float y;
    float theta;
  } g_position;

  struct {
    float x;
    float y;
    float theta;
  } body_speed;

  float linear_speed[3];
  float angular_speed[3];
  union {
    struct {
      uint8_t odom_sensor : 1;
    };
    uint16_t sensor_status;
  };

  float v_bat;
  uint32_t global_flag;
};

send_to_robot send_data;
recv_from_robot recv_data;

robotNode::robotNode() : Node("robot_node") {
  /*IP Config*/
  n_udp.init();
  n_udp.setClient(IP_CLIENT, PORT);

  /*Start Timer*/ //std::chrono::microseconds)100
  udp_timer = this->create_wall_timer((std::chrono::milliseconds)1,
                                      std::bind(&robotNode::udpLoop, this));

  /*Start Publisher*/
  // g_position_pub          =
  // this->create_publisher<geometry_msgs::msg::Pose2D>(
  //                           "global_position", 10);
  // body_speed_pub          =
  // this->create_publisher<geometry_msgs::msg::Twist>(
  //                           "body_speed", 10);
  auto qos = rclcpp::QoS(
      rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history,
                                rmw_qos_profile_sensor_data.history),
      rmw_qos_profile_sensor_data);
  odometry_pub = this->create_publisher<nav_msgs::msg::Odometry>("odom", qos);
  imu_pub = this->create_publisher<sensor_msgs::msg::Imu>("odom/imu", qos);
  cmd_base_vel = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      std::bind(&robotNode::speedSub, this, std::placeholders::_1));
  // run_status_sub          = this->create_subscription<std_msgs::msg::Bool> (
  //                           "run_status", 10,
  //                           std::bind(&robotNode::robot_status_callback,
  //                           this, std::placeholders::_1));
  // seedling_sub            =
  // this->create_subscription<std_msgs::msg::Float32MultiArray> (
  //                           "seedling", 10,
  //                           std::bind(&robotNode::gripper_callback, this,
  //                           std::placeholders::_1));
  // odom_reset_sub          = this->create_subscription<std_msgs::msg::Bool> (
  //                           "odometry/reset", 10,
  //                           std::bind(&robotNode::odomResetSub, this,
  //                           std::placeholders::_1));
  odom_reset_service = this->create_service<std_srvs::srv::Empty>(
      "odom/reset", std::bind(&robotNode::odomResetSub, this,
                              std::placeholders::_1, std::placeholders::_2));

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

/**
 * Fungsi ini akan terpanggil setiap 10us, sesuai dengan settingan variabel
 * timer di constructor class ini
 */
int robotNode::udpLoop() {
  int ret = n_udp.receive();
  if (ret > 0) {  // jika tidak ada atau error, ret == -1
    // RCLCPP_INFO(this->get_logger(), "OK");
    // std::cout << ret << " OK!" << std::endl;

    // Salin data dari array data yang diterima lewat udp ke data yang
    // terstruktur
    memcpy((uint8_t *)&recv_data, n_udp.rx_buff, sizeof(recv_from_robot));

    // Kirim balasan ke udp
    n_udp.send((uint8_t *)&send_data, sizeof(send_to_robot));

    auto odom = nav_msgs::msg::Odometry();

    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.header.stamp = this->get_clock()->now();

    auto q = tf2::Quaternion();
    q.setRPY(0, 0, recv_data.g_position.theta);
    auto pose = geometry_msgs::msg::Pose();
    pose.orientation = tf2::toMsg(q);
    pose.position.x = recv_data.g_position.x;
    pose.position.y = recv_data.g_position.y;
    pose.position.z = 0;

    odom.pose.pose = pose;
    odom.twist.twist.linear.x = recv_data.body_speed.y;
    odom.twist.twist.linear.y = -recv_data.body_speed.x;
    odom.twist.twist.linear.z = 0;
    odom.twist.twist.angular.x = 0;
    odom.twist.twist.angular.y = 0;
    odom.twist.twist.angular.z = recv_data.angular_speed[2];
    // odom.pose.covariance = {{
    //     1e-3, 0, 0, 0, 0, 0,
    //     0, 1e-3, 0, 0, 0, 0,
    //     0, 0, 1e6, 0, 0, 0,
    //     0, 0, 0, 1e6, 0, 0,
    //     0, 0, 0, 0, 1e6, 0,
    //     0, 0, 0, 0, 0, 1e3
    // }};
    // set covariance
    odom.pose.covariance[0] = 0.00001;
    odom.pose.covariance[7] = 0.00001;
    odom.pose.covariance[14] = 1000000000000.0;
    odom.pose.covariance[21] = 1000000000000.0;
    odom.pose.covariance[28] = 1000000000000.0;
    odom.pose.covariance[35] = 0.001;
    


    auto imu = sensor_msgs::msg::Imu();
    imu.header.frame_id = "imu";
    imu.header.stamp = this->get_clock()->now();
    imu.linear_acceleration.x = recv_data.linear_speed[0];
    imu.linear_acceleration.y = recv_data.linear_speed[1];
    imu.linear_acceleration.z = recv_data.linear_speed[2];
    imu.angular_velocity.x = recv_data.angular_speed[0];
    imu.angular_velocity.y = recv_data.angular_speed[1];
    imu.angular_velocity.z = recv_data.angular_speed[2];
    imu.orientation = tf2::toMsg(q);
    
    imu.orientation_covariance[0] = 0.01;
    imu.orientation_covariance[4] = 0.01;
    imu.orientation_covariance[8] = 0.01;
    imu.angular_velocity_covariance[0] = 0.01;
    imu.angular_velocity_covariance[4] = 0.01;
    imu.angular_velocity_covariance[8] = 0.01;
    imu.linear_acceleration_covariance[0] = 0.01;
    imu.linear_acceleration_covariance[4] = 0.01;
    imu.linear_acceleration_covariance[8] = 0.01;

    // geometry_msgs::msg::TransformStamped t;
    // t.set__header(odom.header);
    // t.set__child_frame_id(odom.child_frame_id);
    // t.transform.translation.x = recv_data.g_position.x;
    // t.transform.translation.y = recv_data.g_position.y;
    // t.transform.translation.z = 0;
    // t.transform.set__rotation(tf2::toMsg(q));

    // tf_broadcaster_->sendTransform(t);
    imu_pub->publish(imu);
    odometry_pub->publish(odom);
  }
  return 0;
}

/**
 * Fungsi ini akan terpanggil jika ada data yang dipublish pada topik swerve
 * vector
 */
// int robotNode::swerveSub(const robot_itsrobocon_msgs::msg::PolarVectorArray
// &msg) {
//     // msg.r.size();
//     if(msg.num == 3) {
//         send_data.swerve_speed[0].r = msg.r.at(0);
//         send_data.swerve_speed[1].r = msg.r.at(1);
//         send_data.swerve_speed[2].r = msg.r.at(2);
//         send_data.swerve_speed[0].theta = msg.theta.at(0);
//         send_data.swerve_speed[1].theta = msg.theta.at(1);
//         send_data.swerve_speed[2].theta = msg.theta.at(2);
//     }
//     return 0;
// }

/**
 * Fungsi ini akan terpanggil jika ada data yang dipublish pada topik flag
 */
int robotNode::flagSub(const std_msgs::msg::UInt32 &msg) {
  send_data.flag = msg.data;
}

/**
 * Fungsi ini akan terpanggil jika ada data yang dipublish pada topik swerve
 * vector
 */
int robotNode::speedSub(const geometry_msgs::msg::Twist &msg) {
  send_data.body_speed.x = msg.linear.x;
  send_data.body_speed.y = msg.linear.y;
  send_data.body_speed.theta = msg.angular.z;
}

/**
 * Fungsi ini akan terpanggil jika ada data yang dipublish pada topic run_status
 */
void robotNode::robot_status_callback(const std_msgs::msg::Bool &msg) {
  //@TODO: Program force stop saat msg=false
}

/**
 * Fungsi ini akan terpanggil jika ada data yang dipublish pada topic run_status
 */
// void robotNode::gripper_callback(const std_msgs::msg::Float32MultiArray &msg)
// {
//     send_data.pick_speed = msg.data.at(0);
//     send_data.conveyor_speed = msg.data.at(1);
//     send_data.gripper_pos = msg.data.at(2);
//     send_data.gripper_lift_pos = msg.data.at(3);
//     send_data.gripper_claw = (msg.data.at(4) != 0);
// }

/**
 *
 */
// void robotNode::odomResetSub(const std_msgs::msg::Bool &msg) {
//     send_data.reset_odom = (msg.data)?1:0;
// }
void robotNode::odomResetSub(
    const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    std::shared_ptr<std_srvs::srv::Empty::Response> res) {
  req.get();
  res.get();
  send_data.reset_odom = 1;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robotNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
