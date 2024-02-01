#include "robotname_hardware/robot_node.hpp"
#include "robotname_hardware/udp.hpp"
#include <vector>
#include <bitset>

#define IP_SERVER "192.168.1.111"
#define IP_CLIENT "192.168.1.134"
#define PORT      12345

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

struct send_to_robot{
    uint32_t timestamp;

    struct {
        float r;
        float theta;
    } swerve_speed[NUM_SWERVE];

    float pick_speed;
    float conveyor_speed;

    float gripper_pos;
    float gripper_lift_pos;

    struct {
        float x;
        float y;
        float theta;
    } body_speed;

    union {
		struct {
			uint8_t reset_odom  : 1;
            uint8_t gripper_claw : 1;
		};
		uint32_t flag;
	};
};

struct recv_from_robot{
	uint32_t timestamp;

    struct {
        int32_t x;
        int32_t y;
        float theta;
    } g_position;

    struct {
        float x;
        float y;
        float theta;
    } body_speed;

	//Swerve
    struct {
        float r;
        float theta;
    } swerve[NUM_SWERVE];
    float swerve_drive_torque[NUM_SWERVE];

    union {
    	struct {
    		uint8_t seedling_sensor	: 1;
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

    /*Start Timer*/
    udp_timer = this->create_wall_timer((std::chrono::microseconds)100, std::bind(&robotNode::udpLoop, this));

    /*Start Publisher*/
    // g_position_pub          = this->create_publisher<geometry_msgs::msg::Pose2D>(
    //                           "global_position", 10);
    // body_speed_pub          = this->create_publisher<geometry_msgs::msg::Twist>(
    //                           "body_speed", 10);
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, rmw_qos_profile_sensor_data.history), rmw_qos_profile_sensor_data);
    odometry_pub            = this->create_publisher<nav_msgs::msg::Odometry>(
                              "odometry", qos);
    act_swerve_pub          = this->create_publisher<robot_itsrobocon_msgs::msg::Swerve>(
                              "swerve", 5);
    // act_swerve_current_pub  = this->create_publisher<std_msgs::msg::Float32MultiArray>(
    //                           "swerve/current", 5);

    /*Start Subscriber*/
    cmd_swerve_vector_sub   = this->create_subscription<robot_itsrobocon_msgs::msg::PolarVectorArray> (
                              "cmd_swerve_vel", 10, std::bind(&robotNode::swerveSub, this, std::placeholders::_1));
    cmd_flag_sub            = this->create_subscription<std_msgs::msg::UInt32> (
                              "cmd_flag", 50, std::bind(&robotNode::flagSub, this, std::placeholders::_1));
    cmd_base_vel            = this->create_subscription<geometry_msgs::msg::Twist> (
                              "base_vel", 10, std::bind(&robotNode::speedSub, this, std::placeholders::_1));
    run_status_sub          = this->create_subscription<std_msgs::msg::Bool> (
                              "run_status", 10, std::bind(&robotNode::robot_status_callback, this, std::placeholders::_1));
    seedling_sub            = this->create_subscription<std_msgs::msg::Float32MultiArray> (
                              "seedling", 10, std::bind(&robotNode::gripper_callback, this, std::placeholders::_1));
    // odom_reset_sub          = this->create_subscription<std_msgs::msg::Bool> (
    //                           "odometry/reset", 10, std::bind(&robotNode::odomResetSub, this, std::placeholders::_1));
    odom_reset_service      = this->create_service<std_srvs::srv::Empty>(
                              "odometry/reset", std::bind(&robotNode::odomResetSub, this, std::placeholders::_1, std::placeholders::_2));
}

/**
 * Fungsi ini akan terpanggil setiap 10us, sesuai dengan settingan variabel
 * timer di constructor class ini
*/
int robotNode::udpLoop() {
    int ret = n_udp.receive();
    if(ret > 0) { // jika tidak ada atau error, ret == -1
        // RCLCPP_INFO(this->get_logger(), "OK");
        // std::cout << ret << " OK!" << std::endl;

        //Salin data dari array data yang diterima lewat udp ke data yang terstruktur
        memcpy((uint8_t*)&recv_data, n_udp.rx_buff, sizeof(recv_from_robot));

        //Kirim balasan ke udp
        n_udp.send((uint8_t*)&send_data, sizeof(send_to_robot));

        //Tampilkan data yang diterima
        std::cout << "Timestamp: "
                  << recv_data.timestamp << std::endl

                  << "Position: "
                  << recv_data.g_position.x << ','
                  << recv_data.g_position.y << ','
                  << recv_data.g_position.theta << std::endl

                  << "Body speed: "
                  << recv_data.body_speed.x << ','
                  << recv_data.body_speed.y << ','
                  << recv_data.body_speed.theta << std::endl

                  << "Swerve 0: "
                  << recv_data.swerve[0].r << ','
                  << recv_data.swerve[0].theta << std::endl

                  << "Swerve 1: "
                  << recv_data.swerve[1].r << ','
                  << recv_data.swerve[1].theta << std::endl

                  << "Swerve 2: "
                  << recv_data.swerve[2].r << ','
                  << recv_data.swerve[2].theta << std::endl

                  << "Swerve Torque: "
                  << recv_data.swerve_drive_torque[0] << ','
                  << recv_data.swerve_drive_torque[1] << ','
                  << recv_data.swerve_drive_torque[2] << std::endl

                  << "V Battery: "
                  << recv_data.v_bat << std::endl

                  << "Flag: "
                  << std::bitset<32>(recv_data.global_flag) << std::endl << std::endl;

        // //Publish body global position odometry tp topic global_position
        // auto gpos = geometry_msgs::msg::Pose2D();
        // gpos.x = recv_data.g_position.x;
        // gpos.y = recv_data.g_position.y;
        // gpos.theta = recv_data.g_position.theta;
        // g_position_pub->publish(gpos);

        // //Publish body speed odometry tp topic body_speed
        // auto body_speed = geometry_msgs::msg::Twist();
        // body_speed.linear.x = recv_data.body_speed.x;
        // body_speed.linear.y = recv_data.body_speed.y;
        // body_speed.angular.z = recv_data.body_speed.theta;
        // body_speed_pub->publish(body_speed);

        //Publish odometry data
        // auto odom = robot_itsrobocon::msg::Odometry();
        // odom.pose.x = recv_data.g_position.x;
        // odom.pose.y = recv_data.g_position.y;
        // odom.pose.theta = recv_data.g_position.theta;
        // odom.twist.linear.x = recv_data.body_speed.x;
        // odom.twist.linear.y = recv_data.body_speed.y;
        // odom.twist.angular.z = recv_data.body_speed.theta;
        // odometry_pub->publish(odom);

        auto odom = nav_msgs::msg::Odometry();

        odom.child_frame_id = "odom";
        odom.header.frame_id = "base";

        auto q = tf2::Quaternion();
        q.setRPY(0,0,recv_data.g_position.theta);

        auto pose = geometry_msgs::msg::Pose();
        pose.orientation = tf2::toMsg(q);
        pose.position.x = recv_data.g_position.x;
        pose.position.y = recv_data.g_position.y;
        pose.position.z = 0;

        odom.pose.pose = pose;
        odom.twist.twist.linear.x = recv_data.body_speed.x;
        odom.twist.twist.linear.y = recv_data.body_speed.y;
        odom.twist.twist.linear.z = 0;
        odom.twist.twist.angular.x = 0;
        odom.twist.twist.angular.y = 0;
        odom.twist.twist.angular.z = recv_data.body_speed.theta;

        odometry_pub->publish(odom);
        

        auto swerve = robot_itsrobocon_msgs::msg::Swerve();

        //Publish data actual swerve speed to topic act_swerve_vector
        auto act_swerve = robot_itsrobocon_msgs::msg::PolarVectorArray();
        act_swerve.r = {recv_data.swerve[0].r, 
                        recv_data.swerve[1].r, 
                        recv_data.swerve[2].r};;
        act_swerve.theta = {recv_data.swerve[0].theta, 
                            recv_data.swerve[1].theta, 
                            recv_data.swerve[2].theta};
        act_swerve.num = 3;
        swerve.vel = act_swerve;
        // act_swerve_pub->publish(act_swerve);

        //Publish measured swerve current to act_swerve_current
        auto swerve_current = std::vector<float>();
        swerve_current.push_back(recv_data.swerve_drive_torque[0]);
        swerve_current.push_back(recv_data.swerve_drive_torque[1]);
        swerve_current.push_back(recv_data.swerve_drive_torque[2]);

        swerve.current = swerve_current;

        act_swerve_pub->publish(swerve);
        // act_swerve_current_pub->publish(swerve_current);
    }
    return 0;
}

/**
 * Fungsi ini akan terpanggil jika ada data yang dipublish pada topik swerve vector
*/
int robotNode::swerveSub(const robot_itsrobocon_msgs::msg::PolarVectorArray &msg) {
    // msg.r.size();
    if(msg.num == 3) {
        send_data.swerve_speed[0].r = msg.r.at(0);
        send_data.swerve_speed[1].r = msg.r.at(1);
        send_data.swerve_speed[2].r = msg.r.at(2);
        send_data.swerve_speed[0].theta = msg.theta.at(0);
        send_data.swerve_speed[1].theta = msg.theta.at(1);
        send_data.swerve_speed[2].theta = msg.theta.at(2);
    }
    return 0;
}

/**
 * Fungsi ini akan terpanggil jika ada data yang dipublish pada topik flag
*/
int robotNode::flagSub(const std_msgs::msg::UInt32 &msg) {
    send_data.flag = msg.data;
}

/**
 * Fungsi ini akan terpanggil jika ada data yang dipublish pada topik swerve vector
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
void robotNode::gripper_callback(const std_msgs::msg::Float32MultiArray &msg) {
    send_data.pick_speed = msg.data.at(0);
    send_data.conveyor_speed = msg.data.at(1);
    send_data.gripper_pos = msg.data.at(2);
    send_data.gripper_lift_pos = msg.data.at(3);
    send_data.gripper_claw = (msg.data.at(4) != 0);
}

/**
 * 
*/
// void robotNode::odomResetSub(const std_msgs::msg::Bool &msg) {
//     send_data.reset_odom = (msg.data)?1:0;
// }
void robotNode::odomResetSub(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                                   std::shared_ptr<std_srvs::srv::Empty::Response> res) {
    req.get();
    res.get();
    send_data.reset_odom = 1;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<robotNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
