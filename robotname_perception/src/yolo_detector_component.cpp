#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "robotname_perception/visibility_control.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

#include "robotname_msgs/msg/detection.hpp"
#include "robotname_msgs/msg/detection_array.hpp"

#include "rmw/qos_profiles.h"
#include "rclcpp/qos.hpp"

#include "robotname_perception/yolo_openvino.hpp"

#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "image_geometry/pinhole_camera_model.h"

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

namespace robotname_perception
{

    class yoloDetectorComponent : public rclcpp::Node
    {
    public:
        COMPOSITION_PUBLIC

        explicit yoloDetectorComponent(const rclcpp::NodeOptions &options) : Node("yoloDetectorNode", options)
        {
            // initialize yolo model with certain parameter, please change this to ros parameter
            this->declare_parameter("confidencetreshold", 0.8);
            this->declare_parameter("nmstreshold", 0.8);
            this->declare_parameter("scoretreshold", 0.8);
            this->declare_parameter("modelpath", "/home/ahmadjabar/yolov5/models/ilmiv2_openvino_model/ilmiv2.xml");

            double conftreshold, nmstreshold, scoretreshold;
            std::string modelpath;
            this->get_parameter("confidencetreshold",conftreshold);
            this->get_parameter("nmstreshold",nmstreshold);
            this->get_parameter("scoretreshold",scoretreshold);
            this->get_parameter("modelpath",modelpath);

            yolo::config config{conftreshold, nmstreshold, scoretreshold, 640, 480, 640, modelpath};

            yolomodel = std::make_shared<yolo::yolo>(config);

            rclcpp::QoS qos(rclcpp::KeepLast(1), rmw_qos_profile_default);

            rgb_subs.subscribe(this, "/color/image_raw", qos.get_rmw_qos_profile());
            depth_subs.subscribe(this, "/aligned_depth_to_color/image_raw", qos.get_rmw_qos_profile());
            rgb_cam_info_subs.subscribe(this, "/color/camera_info", qos.get_rmw_qos_profile());

            sync_policies = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>>(rgb_subs, depth_subs, rgb_cam_info_subs, 1);
            sync_policies->registerCallback(&yoloDetectorComponent::rgbd_callback, this);

            //annotated_img_pub = this->create_publisher<sensor_msgs::msg::Image>("/annotated_img", 1);
            detection_pub = this->create_publisher<robotname_msgs::msg::DetectionArray>("/objects/raw", 1);
        }

        ~yoloDetectorComponent()
        {
        }

    private:
        /**
         * @brief rgbd callback function, this function used message_filters package
         * to syncronize rgb, depth, and rgb_cam_info into single callback function. 
         * 
         * @param sensor_msgs::msg::Image rgb_image 
         * @param sensor_msgs::msg::Image depth_image 
         * @param sensor_msgs::msg::CameraInfo rgb_cam_info 
         */
        void rgbd_callback(const sensor_msgs::msg::Image::SharedPtr rgb_image, const sensor_msgs::msg::Image::SharedPtr depth_image, const sensor_msgs::msg::CameraInfo::SharedPtr rgb_cam_info)
        {
            cv_bridge::CvImagePtr rgb_ptr, depth_ptr;
            robotname_msgs::msg::DetectionArray objects;

            /*parse rgb_camera_info information into ROS pinhole camera model*/
            realsense_cam_model.fromCameraInfo(rgb_cam_info);

            /* try conversion type from ROS Image type to OpenCV Image type*/
            try
            {
                rgb_ptr = cv_bridge::toCvCopy(rgb_image, rgb_image->encoding);
                depth_ptr = cv_bridge::toCvCopy(depth_image, depth_image->encoding);
            }
            catch (cv_bridge::Exception &e)
            {
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
            std::vector<yolo::detection> result = yolomodel->detect(rgb_ptr->image);
            
            /*Iterate through the inference result*/
            for (auto &element : result)
            {
                cv::Point2d pixel;
                cv::Point3d obj_coor;

                robotname_msgs::msg::Detection object;

                /* Get the center point of the bounding box*/
                pixel.x = element.box.x + (element.box.width/2);
                pixel.y = element.box.y + (element.box.height/2);

                /* Two different Method can be used to obtain depth value from object
                 * 1. centroid z value 
                        - Extract x,y centroid point
                        - Extract z centroid point
                */
                //float depth_at = 0.001 * (depth_ptr->image.at<u_int16_t>(pixel.y, pixel.x));
                //float depth_at = (depth_ptr->image.at<float>(pixel.y, pixel.x));
                /*
                 * 2. Bounding Box z value
                        - Extract roi from object bbox
                        - Extract single median value from bbox
                */
                cv::Mat roi = depth_ptr->image(element.box);
                float depth_at = 0.001 * calculateMedian(roi);

                /* Use deproject from pixel to 3D coordinate*/
                obj_coor = realsense_cam_model.projectPixelTo3dRay(pixel);

                /* Pack information into vision msgs detections*/
                object.header.frame_id = depth_ptr->header.frame_id;
                object.header.stamp = this->get_clock()->now();
                object.classname = yolo::coconame[element.class_id];
                object.score = element.confidence;
                object.point.x = obj_coor.z * depth_at;
                object.point.y = -obj_coor.x * depth_at;
                object.point.z = obj_coor.y * depth_at;

                objects.detections.push_back(object);
            }

            //rgb_ptr->image = yolomodel->annotated_img(rgb_ptr->image, result);
            //annotated_img_pub->publish(*rgb_ptr->toImageMsg());
            detection_pub->publish(objects);
        }

        float calculateMedian(cv::Mat& inputMat) {
            // Convert the matrix to a 1D vector
            std::vector<uint16_t> values;
            if (inputMat.isContinuous()) {
                values.assign(inputMat.datastart, inputMat.dataend);
            } else {
                for (int i = 0; i < inputMat.rows; ++i) {
                    values.insert(values.end(), inputMat.ptr<uint16_t>(i), inputMat.ptr<uint16_t>(i) + inputMat.cols);
                }
            }

            // Sort the vector
            std::sort(values.begin(), values.end());

            // Calculate the median
            size_t size = values.size();
            if (size % 2 == 0) {
                // If even number of elements, take the average of the two middle elements
                return (values[size / 2 - 1] + values[size / 2]) / 2.0;
            } else {
                // If odd number of elements, return the middle element
                return values[size / 2];
            }
        }

        message_filters::Subscriber<sensor_msgs::msg::Image> rgb_subs;
        message_filters::Subscriber<sensor_msgs::msg::Image> depth_subs;
        message_filters::Subscriber<sensor_msgs::msg::CameraInfo> rgb_cam_info_subs;

        image_geometry::PinholeCameraModel realsense_cam_model;
        std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>> sync_policies;
        std::shared_ptr<yolo::yolo> yolomodel;

        rclcpp::Publisher<robotname_msgs::msg::DetectionArray>::SharedPtr detection_pub;
        //rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr annotated_img_pub;
    };

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(robotname_perception::yoloDetectorComponent)
