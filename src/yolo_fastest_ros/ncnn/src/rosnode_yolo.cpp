#include "yolo-fastestv2.h"
#include "sliced_inference.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <iomanip>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc.hpp>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <target_ros_msgs/BoundingBoxes.h>

namespace {
const char* const class_names[] = {
    "spall_light", "spall_medium", "crack", "rust_corrosion",
    "spall_heavy", "water_efflorescence"
};
}

class ImageConverter {
public:
    ImageConverter() : image_transport_(node_) {
        std::string image_topic, depth_topic, model_path, param_path;
        node_.param<std::string>("subscribers/camera_reading/img_topic", image_topic,
                                 "/camera/color/image_raw");
        node_.param<std::string>("subscribers/camera_reading/depth_topic", depth_topic,
                                 "/camera/aligned_depth_to_color/image_raw");
        node_.param<std::string>("yolo/model_reading/model_path", model_path,
                                 "model/wall_defect6-legacy.bin");
        node_.param<std::string>("yolo/model_reading/para_path", param_path,
                                 "model/wall_defect6-legacy.param");
        if (!node_.getParam("yolo/model_reading/class_names", class_names_)) {
            class_names_.assign(std::begin(class_names), std::end(class_names));
        }
        if (class_names_.empty() || class_names_.size() > 256 ||
            std::any_of(class_names_.begin(), class_names_.end(),
                        [](const std::string& value) { return value.empty(); })) {
            throw std::runtime_error("YOLO class_names must contain nonempty class labels");
        }
        std::vector<double> configured_anchors;
        node_.getParam("yolo/model_reading/anchors", configured_anchors);
        const std::vector<float> anchors(configured_anchors.begin(), configured_anchors.end());
        bool use_gpu;
        int input_size;
        float nms, confidence;
        node_.param("yolo/detection_parameters/use_gpu", use_gpu, false);
        node_.param("yolo/detection_parameters/input_size", input_size, 352);
        node_.param("yolo/detection_parameters/nmsThresh", nms, 0.25f);
        node_.param("yolo/detection_parameters/confThresh", confidence, 0.25f);
        node_.param<std::string>("yolo/detection_parameters/mode", inference_options_.mode, "whole");
        node_.param("yolo/detection_parameters/tile_size", inference_options_.tile_size, 512);
        node_.param("yolo/detection_parameters/tile_overlap", inference_options_.tile_overlap, 0.25);
        node_.param("yolo/detection_parameters/global_nms", inference_options_.global_nms, 0.25);
        node_.param("yolo/detection_parameters/max_tiles", inference_options_.max_tiles, 256);
        inference_options_.validate();
        if (input_size != 352 || nms < 0.f || nms > 1.f ||
            confidence < 0.f || confidence > 1.f) {
            throw std::runtime_error("YOLO requires input_size=352 and thresholds in [0,1]");
        }
        if (detector_.loadParams(use_gpu, input_size, nms, confidence,
                                 static_cast<int>(class_names_.size()), anchors) != 0) {
            throw std::runtime_error("Invalid YOLO model configuration");
        }
        if (detector_.loadModel(resolvePath(param_path).c_str(),
                                resolvePath(model_path).c_str()) != 0) {
            throw std::runtime_error("YOLO model could not be loaded");
        }
        image_sub_ = image_transport_.subscribe(image_topic, 1, &ImageConverter::imageCb, this);
        depth_sub_ = image_transport_.subscribe(depth_topic, 1, &ImageConverter::depthCb, this);
        image_pub_ = image_transport_.advertise("/image_converter/output_video", 1);
        objects_pub_ = node_.advertise<target_ros_msgs::BoundingBoxes>("/objects", 10);
        status_pub_ = node_.advertise<std_msgs::Bool>("/detection_status", 10);
        performance_pub_ = node_.advertise<std_msgs::String>("/yolo/performance", 10);
        ROS_INFO("YOLO ready: %zu classes, first=%s, 352x352, confidence=%.2f, mode=%s, tile=%d, overlap=%.2f",
                 class_names_.size(), class_names_.front().c_str(), confidence,
                 inference_options_.mode.c_str(), inference_options_.tile_size, inference_options_.tile_overlap);
    }

private:
    std::string resolvePath(const std::string& path) const {
        if (path.empty() || path.front() == '/') return path;
        return ros::package::getPath("yolo_detector") + "/" + path;
    }

    void depthCb(const sensor_msgs::ImageConstPtr& message) {
        if (message->encoding != sensor_msgs::image_encodings::TYPE_16UC1 &&
            message->encoding != sensor_msgs::image_encodings::TYPE_32FC1) {
            ROS_WARN_THROTTLE(5.0, "Depth must be aligned to color and encoded as 16UC1 or 32FC1");
            return;
        }
        try {
            depth_ = cv_bridge::toCvCopy(message, message->encoding)->image;
            depth_stamp_ = message->header.stamp;
        } catch (const cv_bridge::Exception& error) {
            ROS_ERROR("Depth conversion: %s", error.what());
        }
    }

    double distanceMm(const cv::Rect& box, const cv::Size& image_size,
                      const ros::Time& stamp) const {
        // Distance remains in millimetres, matching the existing BoundingBox message.
        // Zero means missing, stale, invalid or unaligned depth.
        if (depth_.empty() || depth_.size() != image_size ||
            std::abs((stamp - depth_stamp_).toSec()) > 0.2) return 0.0;
        const cv::Rect center(box.x + box.width / 4, box.y + box.height / 4,
                              std::max(1, box.width / 2), std::max(1, box.height / 2));
        double nearest = 10000.0;
        bool valid = false;
        for (int y = center.y; y < center.y + center.height; ++y) {
            for (int x = center.x; x < center.x + center.width; ++x) {
                const double value = depth_.type() == CV_16UC1
                    ? depth_.at<unsigned short>(y, x)
                    : depth_.at<float>(y, x) * 1000.0;
                if (std::isfinite(value) && value > 0.0 && value < nearest) {
                    nearest = value;
                    valid = true;
                }
            }
        }
        return valid ? nearest : 0.0;
    }

    void imageCb(const sensor_msgs::ImageConstPtr& message) {
        cv_bridge::CvImagePtr image;
        try {
            image = cv_bridge::toCvCopy(message, sensor_msgs::image_encodings::BGR8);
        } catch (const cv_bridge::Exception& error) {
            ROS_ERROR("Image conversion: %s", error.what());
            return;
        }
        if (image->image.empty()) return;
        std::vector<TargetBox> boxes;
        crack_inference::Stats inference_stats;
        const auto start = std::chrono::steady_clock::now();
        try {
            if (crack_inference::run(image->image, inference_options_,
                    [this](const cv::Mat& tile, std::vector<TargetBox>& found) {
                        return detector_.detection(tile, found);
                    }, boxes, inference_stats) != 0) {
                ROS_ERROR_THROTTLE(1.0, "NCNN inference failed; check model inputs and outputs");
                return;
            }
        } catch (const std::exception& error) {
            ROS_ERROR_THROTTLE(1.0, "Inference: %s", error.what());
            return;
        }
        const double inference_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - start).count();
        target_ros_msgs::BoundingBoxes result;
        result.header = message->header;
        result.image_header = message->header;
        const cv::Rect bounds(0, 0, image->image.cols, image->image.rows);
        // Keep labels readable when the full camera frame is fitted into rqt.
        const double display_scale = std::max(1.0,
            std::min(image->image.cols, image->image.rows) / 640.0);
        const double font_scale = 0.90 * display_scale;
        const int text_thickness = std::max(2, cvRound(2 * display_scale));
        const int box_thickness = std::max(3, cvRound(3 * display_scale));
        const int padding = std::max(4, cvRound(4 * display_scale));
        const cv::Scalar highlight(0, 220, 255);
        for (const auto& prediction : boxes) {
            if (prediction.cate < 0 || prediction.cate >= static_cast<int>(class_names_.size()) ||
                prediction.x2 <= prediction.x1 || prediction.y2 <= prediction.y1) continue;
            const cv::Rect box = cv::Rect(prediction.x1, prediction.y1,
                                          prediction.x2 - prediction.x1,
                                          prediction.y2 - prediction.y1) & bounds;
            if (box.empty()) continue;
            target_ros_msgs::BoundingBox item;
            item.probability = prediction.score;
            item.xmin = box.x;
            item.ymin = box.y;
            item.xmax = box.x + box.width;
            item.ymax = box.y + box.height;
            item.distance = distanceMm(box, image->image.size(), message->header.stamp);
            item.id = static_cast<int16_t>(result.bounding_boxes.size());
            item.Class = class_names_[prediction.cate];
            result.bounding_boxes.push_back(item);
            cv::rectangle(image->image, box, highlight, box_thickness, cv::LINE_AA);
            const std::string label = item.Class + cv::format(" %.1f%%", prediction.score * 100.f);
            int baseline = 0;
            const cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX,
                                                       font_scale, text_thickness, &baseline);
            const int label_width = std::min(image->image.cols, text_size.width + 2 * padding);
            const int label_height = std::min(image->image.rows,
                                               text_size.height + baseline + 2 * padding);
            const int label_x = std::max(0, std::min(box.x, image->image.cols - label_width));
            int label_y = box.y - label_height - box_thickness;
            if (label_y < 0) {
                label_y = std::max(0, std::min(box.y + box_thickness,
                                              image->image.rows - label_height));
            }
            cv::rectangle(image->image, cv::Rect(label_x, label_y, label_width, label_height),
                          cv::Scalar(16, 16, 16), cv::FILLED);
            cv::putText(image->image, label,
                         cv::Point(label_x + padding, label_y + padding + text_size.height),
                         cv::FONT_HERSHEY_SIMPLEX, font_scale, highlight, text_thickness, cv::LINE_AA);
        }
        // Publish detections even when no depth camera is connected.
        objects_pub_.publish(result);
        std_msgs::Bool status;
        status.data = !result.bounding_boxes.empty();
        status_pub_.publish(status);
        image_pub_.publish(image->toImageMsg());
        const double elapsed = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - start).count();
        const double source_age_ms = message->header.stamp.isZero() ? -1.0 :
            (ros::Time::now() - message->header.stamp).toSec() * 1000.0;
        std::ostringstream metrics;
        metrics.imbue(std::locale::classic());
        metrics << std::fixed << std::setprecision(3)
                << "{\"mode\":\"" << inference_options_.mode << "\",\"width\":" << image->image.cols
                << ",\"height\":" << image->image.rows << ",\"tiles\":" << inference_stats.tiles
                << ",\"whole_passes\":" << inference_stats.whole_passes
                << ",\"candidates_before_merge\":" << inference_stats.candidates_before_merge
                << ",\"boxes\":" << result.bounding_boxes.size()
                << ",\"inference_ms\":" << inference_ms << ",\"processing_ms\":" << elapsed
                << ",\"source_age_ms\":" << source_age_ms << ",\"source_seq\":" << message->header.seq
                << ",\"stamp_sec\":" << message->header.stamp.sec
                << ",\"stamp_nsec\":" << message->header.stamp.nsec << "}";
        std_msgs::String performance;
        performance.data = metrics.str();
        performance_pub_.publish(performance);
        ROS_INFO_THROTTLE(2.0, "YOLO mode=%s tiles=%zu inference=%.2f ms processing=%.2f ms boxes=%zu",
                         inference_options_.mode.c_str(), inference_stats.tiles, inference_ms, elapsed,
                         result.bounding_boxes.size());
    }

    ros::NodeHandle node_;
    image_transport::ImageTransport image_transport_;
    image_transport::Subscriber image_sub_, depth_sub_;
    image_transport::Publisher image_pub_;
    ros::Publisher objects_pub_, status_pub_, performance_pub_;
    std::vector<std::string> class_names_;
    yoloFastestv2 detector_;
    crack_inference::Options inference_options_;
    cv::Mat depth_;
    ros::Time depth_stamp_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "object_detector");
    try {
        ImageConverter converter;
        ros::spin();
    } catch (const std::exception& error) {
        ROS_FATAL("YOLO startup failed: %s", error.what());
        return 1;
    }
    return 0;
}
