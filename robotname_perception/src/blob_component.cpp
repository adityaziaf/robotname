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

#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"


#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

namespace robotname_perception {

class blobDetectorComponent : public rclcpp::Node {
 public:
  COMPOSITION_PUBLIC

  explicit blobDetectorComponent(const rclcpp::NodeOptions &options)
      : Node("yoloDetectorNode", options) {
    // initialize yolo model with certain parameter, please change this to ros
    // parameter
    this->declare_parameter("mintreshold", 10.0);
    this->declare_parameter("maxthreshold", 200.0);
    this->declare_parameter("minarea", 150.0);
    this->declare_parameter("mincircularity",0.8);

    //rclcpp::QoS qos(rclcpp::KeepLast(1), rmw_qos_profile_default);

    rgb_subs.subscribe(this, "/camera/color/image_raw"/*,qos.get_rmw_qos_profile()*/);
    depth_subs.subscribe(this, "/camera/aligned_depth_to_color/image_raw"/*,
                         qos.get_rmw_qos_profile()*/);
    rgb_cam_info_subs.subscribe(this, "/camera/color/camera_info"/*,
                                qos.get_rmw_qos_profile()*/);

    my_sync_ = std::make_shared<approximate_synchronizer>(approximate_policy(1),rgb_subs , depth_subs, rgb_cam_info_subs);
    my_sync_->getPolicy()->setMaxIntervalDuration(rclcpp::Duration(1,0));
    my_sync_->registerCallback(&blobDetectorComponent::rgbd_callback, this);
    
    annotated_img_pub =
    this->create_publisher<sensor_msgs::msg::Image>("/annotated_img", 1);
    detection_pub = this->create_publisher<robotname_msgs::msg::DetectionArray>(
        "/objects/raw", 1);

  }

  ~blobDetectorComponent() {}

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
        rgb_ptr = cv_bridge::toCvCopy(rgb_image, rgb_image->encoding);
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
    // double _minthreshold, _maxthreshold, _minarea, _mincircularity;

    // this->get_parameter("mintreshold", _minthreshold);
    // this->get_parameter("maxthreshold", _maxthreshold);
    // this->get_parameter("minarea", _minarea);
    // this->get_parameter("mincircularity", _mincircularity);

    // Convert the image to HSV color space
    cv::Mat image_blurred_with_3x3_kernel;
    cv::GaussianBlur(rgb_ptr->image, image_blurred_with_3x3_kernel, cv::Size(5, 5), 0);

    cv::Mat hsv;
    cv::cvtColor(image_blurred_with_3x3_kernel, hsv, cv::COLOR_RGB2HSV);

    // Define the lower and upper bounds for the red color (you may need to
    // adjust these)
    cv::Scalar lower_red = cv::Scalar(0, 220, 20);
    cv::Scalar upper_red = cv::Scalar(15, 255, 190);

    // Create a mask using the inRange function to extract only the red pixels
    cv::Mat mask;
    cv::inRange(hsv, lower_red, upper_red, mask);

    cv::GaussianBlur(mask, mask, cv::Size(5, 5), 0);
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(mask,contours, hierarchy, cv::RETR_TREE,cv::CHAIN_APPROX_SIMPLE);

    if(!contours.empty())
    {
      std::vector<cv::Point> largest;
      float area = 0;
      for (auto& it : contours) { 
          float temp_area = cv::contourArea(it);
            if (temp_area > area)
            {
               area = temp_area;
               largest = it;
            }
      }
      auto moment = cv::moments(largest);

      int cx = moment.m10/moment.m00;
      int cy = moment.m01/moment.m00;

      cv::Point2d pixel;
      cv::Point3d obj_coor;

      robotname_msgs::msg::Detection object;

      /* Get the center point of the bounding box*/
      pixel.x = cx;
      pixel.y = cy;

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

      // /* Pack information into vision msgs detections*/
      object.pose.header.frame_id = rgb_ptr->header.frame_id;
      object.pose.header.stamp = rgb_ptr->header.stamp;
      object.id = 0;
      object.classname = "redball";
      object.score = 0.0; //#element.confidence;
      object.pose.pose.position.x = obj_coor.z * depth_at;
      object.pose.pose.position.y = -obj_coor.x * depth_at;
      object.pose.pose.position.z = -obj_coor.y * depth_at;
      object.pose.pose.position.x = obj_coor.x * depth_at;
      object.pose.pose.position.y = obj_coor.y * depth_at;
      object.pose.pose.position.z = obj_coor.z * depth_at;
      object.pose.pose.orientation.x = 0;
      object.pose.pose.orientation.y = 0;
      object.pose.pose.orientation.z = 0;
      object.pose.pose.orientation.w = 1;
      objects->detections.push_back(object);
      //cv::circle(rgb_ptr->image,cv::Point(cx,cy),0.1,(0,255,0),1);
    } 
 
    annotated_img_pub->publish(*rgb_ptr->toImageMsg());
    detection_pub->publish(std::move(objects));
  }

  message_filters::Subscriber<sensor_msgs::msg::Image> rgb_subs;
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_subs;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> rgb_cam_info_subs;

  image_geometry::PinholeCameraModel realsense_cam_model;
  // std::shared_ptr<message_filters::TimeSynchronizer<
  //     sensor_msgs::msg::Image, sensor_msgs::msg::Image,
  //     sensor_msgs::msg::CameraInfo>>
  //     sync_policies;
  //typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo> approximate_policy;
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
RCLCPP_COMPONENTS_REGISTER_NODE(robotname_perception::blobDetectorComponent)
