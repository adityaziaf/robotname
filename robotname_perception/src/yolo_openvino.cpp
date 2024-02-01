#include "robotname_perception/yolo_openvino.hpp"
#include <iostream>
#include <string>
#include <time.h>

namespace yolo
{

/**
 * @brief Construct a new yolo::yolo object
 * 
 * @param yolo::config config 
 */
yolo::yolo(config config) {
	this->confThreshold = config.confThreshold;
	this->nmsThreshold = config.nmsThreshold;
    this->scoreThreshold = config.scoreThreshold;
	this->inpWidth = config.inpWidth;
	this->inpHeight = config.inpHeight;
    this->yolocanvas = config.yolocanvas;
	this->onnx_path = config.onnx_path;
    this->initialmodel();
}
/**
 * @brief Destroy the yolo::yolo object
 * 
 */
yolo::~yolo(){}
/**
 * @brief yolo run inference function
 * 
 * @param cv::Mat frame 
 * @return std::vector<detection> 
 */
std::vector<detection> yolo::detect(cv::Mat & frame) {
    
    this->preprocess_img(frame);
    infer_request.infer();
    const ov::Tensor& output_tensor = infer_request.get_output_tensor();
    ov::Shape output_shape = output_tensor.get_shape();
    float* detections = output_tensor.data<float>();
    return this->postprocess_img(detections, output_shape);

}
/**
 * @brief Init Openvino model parameter, compile onnx model into IR representation
 * 
 */
void yolo::initialmodel() {
    ov::Core core;
    std::shared_ptr<ov::Model> model = core.read_model(this->onnx_path);
    ov::preprocess::PrePostProcessor ppp = ov::preprocess::PrePostProcessor(model);
    ppp.input().tensor().set_element_type(ov::element::u8).set_layout("NHWC").set_color_format(ov::preprocess::ColorFormat::RGB);
    ppp.input().preprocess().convert_element_type(ov::element::f32).convert_color(ov::preprocess::ColorFormat::RGB).scale({ 255, 255, 255 });// .scale({ 112, 112, 112 });
    ppp.input().model().set_layout("NCHW");
    ppp.output().tensor().set_element_type(ov::element::f32);
    model = ppp.build();
    this->compiled_model = core.compile_model(model, "GPU");
    this->infer_request = compiled_model.create_infer_request();

}
/**
 * @brief Do some preprocessing stuff to the image before loaded it into the yolo network,
 * image size defined at constructor
 * 
 * @param cv::Mat frame 
 */
void yolo::preprocess_img(cv::Mat& frame) {
    // float width = frame.cols;
    // float height = frame.rows;
    // cv::Size new_shape= cv::Size(inpWidth, inpHeight);
    // float r = float(new_shape.width / std::max(width, height));
    // int new_unpadW = int(round(width * r));
    // int new_unpadH = int(round(height * r));

    // cv::resize(frame, resize_img.resized_image, cv::Size(new_unpadW, new_unpadH), 0, 0, cv::INTER_AREA);
    // resize_img.resized_image = resize_img.resized_image;
    // resize_img.dw = new_shape.width - new_unpadW;
    // resize_img.dh = new_shape.height - new_unpadH;
    // cv::Scalar color = cv::Scalar(100, 100, 100);
    // cv::copyMakeBorder(resize_img.resized_image, resize_img.resized_image, 0, resize_img.dh, 0, resize_img.dw, cv::BORDER_CONSTANT, color);

    // this->rx = (float)frame.cols / (float)(resize_img.resized_image.cols - resize_img.dw);
    // this->ry = (float)frame.rows / (float)(resize_img.resized_image.rows - resize_img.dh);
    // float* input_data = (float*)resize_img.resized_image.data;

    cv::Mat canvas(yolocanvas, yolocanvas, CV_8UC3, cv::Scalar(255,255,255));
    // Overlay the smaller image onto the larger image
    frame.copyTo(canvas(cv::Rect(0 , 0, frame.cols, frame.rows)));

    float* input_data = (float*)canvas.data;
    input_tensor = ov::Tensor(compiled_model.input().get_element_type(), compiled_model.input().get_shape(), input_data);
    infer_request.set_input_tensor(input_tensor);
}
/**
 * @brief Do some annotation to the image, based on bbox, label, and score information
 * from the inference result
 * 
 * @param cv::mat frame 
 * @param std::vector<detection> output 
 * @return cv::Mat frame
 */
cv::Mat yolo::annotated_img(cv::Mat& frame, std::vector<detection> output)
{
    for (int i = 0; i < output.size(); i++)
    {
        auto detection = output[i];
        auto box = detection.box;
        auto classId = detection.class_id;
        // if (classId != 0) continue;
        auto confidence = detection.confidence;
        
        box.x = this->rx * box.x;
        box.y = this->ry * box.y;
        box.width = this->rx * box.width;
        box.height = this->ry * box.height;
        float xmax = box.x + box.width;
        float ymax = box.y + box.height;
        cv::Scalar color = cv::Scalar(color_list[classId][0], color_list[classId][1], color_list[classId][2]);
        float c_mean = cv::mean(color)[0];
        cv::Scalar txt_color;
        if (c_mean > 0.5) {
            txt_color = cv::Scalar(0, 0, 0);
        }
        else {
            txt_color = cv::Scalar(255, 255, 255);
        }
        cv::rectangle(frame, cv::Point(box.x, box.y), cv::Point(xmax, ymax), color*255,2);
        int baseLine=0;
        char text[512];
        sprintf(text, "%s %0.1f%%", coconame[classId], confidence*100);
        cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &baseLine);
        cv::Scalar txt_bk_color = color * 0.7 * 255;
        cv::rectangle(frame, cv::Rect(cv::Point(box.x, box.y), cv::Size(label_size.width, label_size.height+baseLine)),
         txt_bk_color, -1);
        cv::putText(frame, text, cv::Point(box.x, box.y+label_size.height), cv::FONT_HERSHEY_SIMPLEX, 0.4, txt_color,1);

    }

    return frame;
}
/**
 * @brief Do some post processing to query bbox,label, and score result from the network
 * 
 * @param float* detections 
 * @param ov::Shape output_shape 
 * @return std::vector<detection> 
 */
std::vector<detection> yolo::postprocess_img(float* detections, ov::Shape & output_shape) {
    std::vector<cv::Rect> boxes;
    std::vector<int> class_ids;
    std::vector<float> confidences;
    for (int i = 0; i < output_shape[1]; i++) {
        float* detection = &detections[i * output_shape[2]];

        float confidence = detection[4];
        if (confidence >= this->confThreshold) {
            float* classes_scores = &detection[5];
            cv::Mat scores(1, output_shape[2] - 5, CV_32FC1, classes_scores);
            cv::Point class_id;
            double max_class_score;
            cv::minMaxLoc(scores, 0, &max_class_score, 0, &class_id);
            if (max_class_score > this->scoreThreshold) {
                confidences.push_back(confidence);
                class_ids.push_back(class_id.x);
                float x = detection[0];
                float y = detection[1];
                float w = detection[2];
                float h = detection[3];
                float xmin = x - (w / 2);
                float ymin = y - (h / 2);

                boxes.push_back(cv::Rect(xmin, ymin, w, h));
            }
        }
    }

    std::vector<int> nms_result;
    cv::dnn::NMSBoxes(boxes, confidences, this->scoreThreshold, this->nmsThreshold, nms_result);

    std::vector<detection> output;
    for (int i = 0; i < nms_result.size(); i++)
    {
        detection result;
        int idx = nms_result[i];
        result.class_id = class_ids[idx];
        result.confidence = confidences[idx];
        result.box = boxes[idx];
        output.push_back(result); 
    }

    return output;
}

}
