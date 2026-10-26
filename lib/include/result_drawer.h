#pragma once

#include <vector>

#include <opencv2/core.hpp>

#include "pose_solver.h"

void draw_corners(
    cv::Mat& frame, 
    const std::vector<std::vector<cv::Point2f>>& corners);

void draw_poses(
    cv::Mat& frame,
    const std::vector<std::vector<cv::Point2f>>& corners,
    const std::vector<PoseInfo>& poses);