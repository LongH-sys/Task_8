#pragma once

#include <vector>

#include <opencv2/core.hpp>

struct PoseInfo
{
    double yaw_deg = 0.0;
    double pitch_deg = 0.0;
    double raw_deg = 0.0;
    double distance_mm = 0.0;
};

bool estimate_pose(
    const std::vector<cv::Point2f>& corners, 
    PoseInfo& pose);