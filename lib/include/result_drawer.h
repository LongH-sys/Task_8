#pragma once

#include <vector>

#include <opencv2/core.hpp>

void draw_corners(
    cv::Mat &frame, 
    const std::vector<cv::Point2f>& corners);

void draw_pose(
    cv::Mat &frame,
    const PoseInfo& pose);