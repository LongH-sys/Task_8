#pragma once

#include <vector>

#include <opencv2/core.hpp>

struct PoseInfo
{
    int index = 0;
    double yaw_deg = 0.0;
    double pitch_deg = 0.0;
    double roll_deg = 0.0;
    double distance_mm = 0.0;
};

struct validArmor
{
    int index = 0;
    cv::Vec3d rvec;
    cv::Vec3d tvec;
};

bool estimate_pose(
    const std::vector<std::vector<cv::Point2f>>& corners, 
    std::vector<PoseInfo>& poses);