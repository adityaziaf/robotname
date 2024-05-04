#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "cv_bridge/cv_bridge.h"
#include "image_geometry/pinhole_camera_model.h"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/time_synchronizer.h"
#include "opencv2/opencv.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rmw/qos_profiles.h"
#include "robotname_msgs/msg/detection.hpp"
#include "robotname_msgs/msg/detection_array.hpp"
#include "robotname_perception/visibility_control.h"
#include "robotname_perception/yolo_onnx.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace robotname_perception {

class yoloOnnxComponent : public rclcpp::Node {
 public:
  COMPOSITION_PUBLIC

  explicit yoloOnnxComponent(const rclcpp::NodeOptions &options)
      : Node("yoloOnnxNode", options) {
    // initialize yolo model with certain parameter, please change this to ros
    // parameter
    this->declare_parameter("confidencetreshold", 0.8);
    this->declare_parameter("nmstreshold", 0.8);
    this->declare_parameter("scoretreshold", 0.8);
    this->declare_parameter(
        "modelpath",
        "/home/ahmadjabar/yolov5/models/omni.onnx");

    double conftreshold, nmstreshold, scoretreshold;
  
    this->get_parameter("confidencetreshold", conftreshold);
    this->get_parameter("nmstreshold", nmstreshold);
    this->get_parameter("scoretreshold", scoretreshold);

    const std::string modelpath = 
    ament_index_cpp::get_package_share_directory("robotname_perception") + "/config/omni.onnx";
    const std::string classnames = 
    ament_index_cpp::get_package_share_directory("robotname_perception") + "/config/ball.names";

    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/omni/image_raw", 1, std::bind(&yoloOnnxComponent::topic_callback, this, std::placeholders::_1));

    annotated_img_pub =
    this->create_publisher<sensor_msgs::msg::Image>("/omni/annotated_img", 1);

    detection_pub = this->create_publisher<robotname_msgs::msg::DetectionArray>(
        "/omni/objects/raw", 1);

    NetConfig DetectorConfig = {0.5, 0.5, modelpath, classnames};
    net = std::make_unique<YOLODetector>(DetectorConfig);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  }

  ~yoloOnnxComponent() {}

 private:
  /**
   * @brief rgbd callback function, this function used message_filters package
   * to syncronize rgb, depth, and rgb_cam_info into single callback function.
   *
   * @param sensor_msgs::msg::Image rgb_image
   * @param sensor_msgs::msg::Image depth_image
   * @param sensor_msgs::msg::CameraInfo rgb_cam_info
   */
  void topic_callback(
      const sensor_msgs::msg::Image::SharedPtr rgb_image) {
    cv_bridge::CvImagePtr rgb_ptr;

    /* try conversion type from ROS Image type to OpenCV Image type*/
    try {
      rgb_ptr = cv_bridge::toCvCopy(rgb_image, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
    
    auto result = net->Detect(rgb_ptr->image);

    /*Iterate through the inference result*/

    robotname_msgs::msg::DetectionArray objects;

    try {
        geometry_msgs::msg::TransformStamped transform =
   tf_buffer_->lookupTransform("base_link",
                              "omnicam_link",
                              tf2::TimePointZero);
    geometry_msgs::msg::TransformStamped transform_map =
   tf_buffer_->lookupTransform("map",
                              "base_link",
                              tf2::TimePointZero);

    for(auto &element : result)
    {
      cv::Point2d pixel;
      cv::Point3d obj_coor;
      robotname_msgs::msg::Detection object;

         /* Get the center point of the bounding box*/
        pixel.x = -element.cx;
        pixel.y = element.cy;

        float distancex = pixel.x + 313; //center x
        float distancey = pixel.y - 230; //center y

        float objdistance = std::hypot(distancex,distancey);
        float theta = std::atan2(distancey, distancex);

        float real_dist = 0.004310*objdistance - 0.475;
        if (real_dist < 0.0) real_dist = 0.0;             
        
        //omnicam_link to base_link
        geometry_msgs::msg::PoseStamped poseInSourceFrame, poseInTargetFrame,poseinTargetMap;
        poseInSourceFrame.pose.position.x = real_dist*sin(theta);
        poseInSourceFrame.pose.position.y = real_dist*cos(theta);
        poseInSourceFrame.pose.position.z = 0.0;
        poseInSourceFrame.header.frame_id = "omnicam_link";
        tf2::doTransform(poseInSourceFrame, poseInTargetFrame, transform);
        
        tf2::Quaternion quat;
        quat.setRPY(0.0, 0.0, std::atan2(poseInTargetFrame.pose.position.y, poseInTargetFrame.pose.position.x));
        poseInTargetFrame.pose.set__orientation(tf2::toMsg(quat)); // Roll, pitch, yaw
        tf2::doTransform(poseInTargetFrame, poseinTargetMap, transform_map);

         /* Pack information into vision msgs detections*/
                
        object.classname = net->classNames[element.label];
        object.pixelx = pixel.x;
        object.pixely = pixel.y;
        object.score = element.score;
        object.pose.set__header(poseinTargetMap.header);
        object.pose.set__pose(poseinTargetMap.pose);
       

        objects.detections.push_back(object);
    }
      } catch (const tf2::TransformException &ex) {
        RCLCPP_INFO(this->get_logger(), "Could not transform");
                    return;
      }
                              
    
    
    detection_pub->publish(objects);
    // rgb_ptr->image = yolomodel->annotated_img(rgb_ptr->image, result);
    net->DrawBoxes(rgb_ptr->image, result);
    annotated_img_pub->publish(*rgb_ptr->toImageMsg());
    
  }

  // message_filters::Subscriber<sensor_msgs::msg::Image> rgb_subs;
  // message_filters::Subscriber<sensor_msgs::msg::Image> depth_subs;
  // message_filters::Subscriber<sensor_msgs::msg::CameraInfo> rgb_cam_info_subs;

  std::unique_ptr<YOLODetector> net;
  // image_geometry::PinholeCameraModel realsense_cam_model;
  // std::shared_ptr<message_filters::TimeSynchronizer<
  //     sensor_msgs::msg::Image, sensor_msgs::msg::Image,
  //     sensor_msgs::msg::CameraInfo>>
  //     sync_policies;

  // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo> approximate_policy;
  // typedef message_filters::Synchronizer<approximate_policy> approximate_synchronizer;
  // std::shared_ptr<approximate_synchronizer> my_sync_;

  rclcpp::Publisher<robotname_msgs::msg::DetectionArray>::SharedPtr
      detection_pub;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr annotated_img_pub;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  geometry_msgs::msg::TransformStamped transform;

};

}  // namespace robotname_perception

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(robotname_perception::yoloOnnxComponent)
