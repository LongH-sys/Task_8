#include "pose_solver.h"

#include <opencv2/calib3d.hpp>

#include <cmath>

#include <algorithm>

bool estimate_pose(
    const std::vector<std::vector<cv::Point2f>>& corners,
    std::vector<PoseInfo>& poses)
{
    poses.clear();
    float width = 135.0f;
    float height = 55.0f;
    std::vector<cv::Point3f> objectPoints =                   
    {
        {-width / 2.0f, -height / 2.0f, 0.0f},
        {-width / 2.0f, height / 2.0f, 0.0f},
        {width / 2.0f, -height / 2.0f, 0.0f},
        {width / 2.0f, height / 2.0f, 0.0f}
    };                    
    double fx = 1811.899787;
    double fy = 1810.996967;
    double cx = 743.1573263;
    double cy = 575.0848947;
    cv::Mat cameraMatrix = (cv::Mat_<double>(3,3) <<
        fx, 0, cx,
        0, fy, cy,
        0,  0,  1
    );
    double k1 = -0.087493626;
    double k2 = 0.2084118559;
    cv::Mat distCoeffs = (cv::Mat_<double>(1,5) <<
        k1, k2, 0, 0, 0        
    );
    std::vector<validArmor> validPoses;
    for (int i = 0; i < static_cast<int>(corners.size()); i++)
    {
        cv::Vec3d rvec;
        cv::Vec3d tvec;
        bool success = cv::solvePnP(
            objectPoints,        
            corners[i],
            cameraMatrix,
            distCoeffs,
            rvec,
            tvec,
            false,
            cv::SOLVEPNP_IPPE
        );
        if (!success)
        {
            continue;
        }
        else
        {
            validPoses.push_back({i, rvec, tvec});
        }
    }
    for (int i = 0; i < static_cast<int>(validPoses.size()); i++)
    {
        int index = validPoses[i].index;
        double distance = cv::norm(validPoses[i].tvec);
        cv::Mat R, R64;
        cv::Rodrigues(validPoses[i].rvec, R);
        R.convertTo(R64, CV_64F);
        double yaw = std::atan2(R64.at<double>(0, 2), R64.at<double>(2, 2)) * 180.0 / CV_PI;
        double pitch = -std::asin(std::clamp(R64.at<double>(1, 2), -1.0, 1.0)) * 180.0 / CV_PI;
        double roll = std::atan2(R64.at<double>(1, 0), R64.at<double>(1, 1)) * 180.0 / CV_PI;
        poses.push_back({index, yaw, pitch, roll, distance});
    }
    return !poses.empty();
}