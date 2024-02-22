#include "opencv2/opencv.hpp"
#include "robotname_perception/yolo_openvino.hpp"

int main()
{
    cv::Mat image;
    cv::namedWindow("display");
    cv::VideoCapture cap(2);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    yolo::config cfg{0.85, 0.85, 0.85, 640, 480, 640, "/home/ahmadjabar/yolov5/models/ilmiv2_openvino_model/ilmiv2.xml"};
    yolo::yolo detector(cfg);
    if(!cap.isOpened()) {
        std::cout << "cannot open camera";
    }
    
    while(true)
    {
        cap >> image;
        std::vector<yolo::detection> detect_list = detector.detect(image);
        detector.annotated_img(image, detect_list);
        cv::imshow("display",image);
        cv::waitKey(25);
    }
    return 0;
}