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
        "/home/ahmadjabar/yolov5/models/ilmiv2.onnx");

    double conftreshold, nmstreshold, scoretreshold;
  
    this->get_parameter("confidencetreshold", conftreshold);
    this->get_parameter("nmstreshold", nmstreshold);
    this->get_parameter("scoretreshold", scoretreshold);

    const std::string modelpath = 
    ament_index_cpp::get_package_share_directory("robotname_perception") + "/config/ilmiv2.onnx";
    const std::string classnames = 
    ament_index_cpp::get_package_share_directory("robotname_perception") + "/config/ball.names";

    rclcpp::QoS qos(rclcpp::KeepLast(1), rmw_qos_profile_default);

    // rgb_subs.subscribe(this, "/camera/color/image_raw", qos.get_rmw_qos_profile());
    // depth_subs.subscribe(this, "/camera/aligned_depth_to_color/image_raw",
    //                      qos.get_rmw_qos_profile());
    // rgb_cam_info_subs.subscribe(this, "/camera/color/camera_info",
    //                             qos.get_rmw_qos_profile());

    // sync_policies = std::make_shared<message_filters::TimeSynchronizer<
    //     sensor_msgs::msg::Image, sensor_msgs::msg::Image,
    //     sensor_msgs::msg::CameraInfo>>(rgb_subs, depth_subs, rgb_cam_info_subs,
    //                                    1);
    // sync_policies->registerCallback(&yoloOnnxComponent::rgbd_callback, this);

    rgb_subs.subscribe(this, "/camera/color/image_raw"/*,qos.get_rmw_qos_profile()*/);
    depth_subs.subscribe(this, "/camera/aligned_depth_to_color/image_raw"/*,
                         qos.get_rmw_qos_profile()*/);
    rgb_cam_info_subs.subscribe(this, "/camera/color/camera_info"/*,
                                qos.get_rmw_qos_profile()*/);

    my_sync_ = std::make_shared<approximate_synchronizer>(approximate_policy(1),rgb_subs , depth_subs, rgb_cam_info_subs);
    my_sync_->getPolicy()->setMaxIntervalDuration(rclcpp::Duration(1,0));
    my_sync_->registerCallback(&yoloOnnxComponent::rgbd_callback, this);

    annotated_img_pub =
    this->create_publisher<sensor_msgs::msg::Image>("/annotated_img", 1);
    detection_pub = this->create_publisher<robotname_msgs::msg::DetectionArray>(
        "/objects/raw", 1);

    NetConfig DetectorConfig = {0.5, 0.5, modelpath, classnames};
    net = std::make_unique<YOLODetector>(DetectorConfig);
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
  void rgbd_callback(
      const sensor_msgs::msg::Image::SharedPtr rgb_image,
      const sensor_msgs::msg::Image::SharedPtr depth_image,
      const sensor_msgs::msg::CameraInfo::SharedPtr rgb_cam_info) {
    cv_bridge::CvImagePtr rgb_ptr, depth_ptr;

    std::unique_ptr<robotname_msgs::msg::DetectionArray> objects(
        new robotname_msgs::msg::DetectionArray());

    /*parse rgb_camera_info information into ROS pinhole camera model*/
    realsense_cam_model.fromCameraInfo(rgb_cam_info);

    /* try conversion type from ROS Image type to OpenCV Image type*/
    try {
      rgb_ptr = cv_bridge::toCvCopy(rgb_image, sensor_msgs::image_encodings::BGR8);
      depth_ptr = cv_bridge::toCvCopy(depth_image, depth_image->encoding);
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }

    /* in case of '16UC1' encoding (depth values are in millimeter),
     * a manually conversion from millimeter to meter is required.
     */
    // if(depth_ptr->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
    // {
    //     depth_ptr->image.convertTo(depth_ptr->image, -1, 0.001f);
    // }

    /* Run YOLO Inference */
    // std::vector<yolo::detection> result = yolomodel->detect(rgb_ptr->image);
    // cv::Mat canvas(640, 640, CV_8UC3, cv::Scalar(255,255,255));
    // // Overlay the smaller image onto the larger image
    // rgb_ptr->image.copyTo(canvas(cv::Rect(0 , 0, rgb_ptr->image.cols, rgb_ptr->image.rows)));
    auto result = net->Detect(rgb_ptr->image);

    for (auto &element : result) {
      cv::Point2d pixel;
      cv::Point3d obj_coor;

      robotname_msgs::msg::Detection object;

      /* Get the center point of the bounding box*/
      pixel.x = element.cx;
      pixel.y = element.cy;

      /* Two different Method can be used to obtain depth value from object
       * 1. centroid z value
              - Extract x,y centroid point
              - Extract z centroid point
      */
      float depth_at =
          0.001 * (depth_ptr->image.at<u_int16_t>(pixel.y, pixel.x));
      // float depth_at = (depth_ptr->image.at<float>(pixel.y, pixel.x));
      /*
       * 2. Bounding Box z value
              - Extract roi from object bbox
              - Extract single median value from bbox
      */
      // cv::Mat roi = depth_ptr->image(element.box);
      // float depth_at = 0.001 * calculateMedian(roi);

      /* Use deproject from pixel to 3D coordinate*/
      obj_coor = realsense_cam_model.projectPixelTo3dRay(pixel);

      /* Pack information into vision msgs detections*/
      object.pose.header.frame_id = rgb_ptr->header.frame_id;
      object.pose.header.stamp = rgb_ptr->header.stamp;
      object.classname = net->classNames[element.label];
      object.score = element.score;
      // object.pose.pose.position.x = obj_coor.z * depth_at;
      // object.pose.pose.position.y = -obj_coor.x * depth_at;
      // object.pose.pose.position.z = -obj_coor.y * depth_at;
      object.pose.pose.position.x = obj_coor.x * depth_at;
      object.pose.pose.position.y = obj_coor.y * depth_at;
      object.pose.pose.position.z = obj_coor.z * depth_at;
      object.pose.pose.orientation.x = 0;
      object.pose.pose.orientation.y = 0;
      object.pose.pose.orientation.z = 0;
      object.pose.pose.orientation.w = 1;
      objects->detections.push_back(object);
    }
    // rgb_ptr->image = yolomodel->annotated_img(rgb_ptr->image, result);
    net->DrawBoxes(rgb_ptr->image, result);
    annotated_img_pub->publish(*rgb_ptr->toImageMsg());
    detection_pub->publish(std::move(objects));
  }

  message_filters::Subscriber<sensor_msgs::msg::Image> rgb_subs;
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_subs;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> rgb_cam_info_subs;

  std::unique_ptr<YOLODetector> net;
  image_geometry::PinholeCameraModel realsense_cam_model;
  // std::shared_ptr<message_filters::TimeSynchronizer<
  //     sensor_msgs::msg::Image, sensor_msgs::msg::Image,
  //     sensor_msgs::msg::CameraInfo>>
  //     sync_policies;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo> approximate_policy;
  typedef message_filters::Synchronizer<approximate_policy> approximate_synchronizer;
  std::shared_ptr<approximate_synchronizer> my_sync_;

  rclcpp::Publisher<robotname_msgs::msg::DetectionArray>::SharedPtr
      detection_pub;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr annotated_img_pub;
};

}  // namespace robotname_perception

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(robotname_perception::yoloOnnxComponent)
