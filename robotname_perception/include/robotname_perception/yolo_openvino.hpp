#pragma once

#include <string>
#include <iostream>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <openvino/openvino.hpp>

namespace yolo
{

const char* coconame[] = {"blueball", "purpleball", "redball"};

const float color_list[3][3] =
{
    {0.000, 0.447, 0.741},
    {0.000, 1.000, 0.000},
    {0.000, 1.000, 0.500}
};

struct config
{
    double confThreshold;
    double nmsThreshold;
    double scoreThreshold;
    int inpWidth;
    int inpHeight;
    int yolocanvas;
    std::string onnx_path;
};

struct resize
{
    cv::Mat resized_image;
    int dw;
    int dh;
};

struct detection
{
    int class_id;
    float confidence;
    cv::Rect box;
    float point[3];
};

class yolo
{
public:
    yolo(config config);
    ~yolo();

    std::vector<detection> detect(cv::Mat &frame);
    void preprocess_img(cv::Mat &frame);
    cv::Mat annotated_img(cv::Mat &frame, std::vector<detection>);
    std::vector<detection> postprocess_img(float *detections, ov::Shape &output_shape);

private:
    float confThreshold;
    float nmsThreshold;
    float scoreThreshold;
    int inpWidth;
    int inpHeight;
    int yolocanvas;
    float rx; // the width ratio of original image and resized image
    float ry; // the height ratio of original image and resized image
    std::string onnx_path;
    resize resize_img;
    ov::Tensor input_tensor;
    ov::InferRequest infer_request;
    ov::CompiledModel compiled_model;

    void initialmodel();
};

}
