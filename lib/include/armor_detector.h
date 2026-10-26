#pragma once

#include <vector>

#include <opencv2/core.hpp>

#include <yolo_vino.hpp>

bool detect_corners(
    const cv::Mat& frame,
    std::vector<std::vector<cv::Point2f>>& corners, 
    YoloVino::Yolov8poseVino& detector);