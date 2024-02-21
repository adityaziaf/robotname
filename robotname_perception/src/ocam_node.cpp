#include "omnicamera_model/ocam.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "opencv2/opencv.hpp"

const std::string ocam_param = 
    ament_index_cpp::get_package_share_directory("robotname_perception") + "/config/ocam_param.txt";

int main()
{
    omni_cam::OCamPtr cam;
    cam = omni_cam::OCam::loadOCam(ocam_param);
    

    cv::Mat image;

    cv::namedWindow("Display window");

    cv::VideoCapture cap(0);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    if (!cap.isOpened()) {

    std::cout << "cannot open camera";

    }

    while (true) {

    cap >> image;
    const Eigen::Vector2d keypoint(400.0, 300.0);
    Eigen::Vector3d bound;
    cam->backProject3(keypoint, &bound);
    std::cout << bound.x() << " " << bound.y() << " " << bound.z() << std::endl;
    imshow("Display window", image);

    cv::waitKey(25);

    }

    return 0;

}