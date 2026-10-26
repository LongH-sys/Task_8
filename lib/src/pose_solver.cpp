#include "pose_solver.h"

#include <opencv2/calib3d.hpp>

#include <cmath>

#include <algorithm>

bool estimate_pose(
    const std::vector<std::vector<cv::Point2f>>& corners,
    std::vector<PoseInfo>& poses)
{
    poses.clear();
    float width =                                             //静待装甲板的尺寸
    float height = 
    std::vector<cv::Point3f> objectPoints =                   
    {
        {-width / 2.0f, -height / 2.0f, 0.0f},
        {width / 2.0f, -height / 2.0f, 0.0f},
        {width / 2.0f, height / 2.0f, 0.0f},
        {-width / 2.0f, height / 2.0f, 0.0f}
    };                    
    cv::Mat cameraMatrix = (cv::Mat_<double>(3,3) <<          //静待相机标定 
        fx, 0, cx,
        0, fy, cy,
        0,  0,  1
    );
    cv::Mat distCoeffs = (cv::Mat_<double>(1,5) <<            //静待相机标定
        k1, k2, p1, p2, k3        
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
            cv::SOLVEPNP_EPNP
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
        double yaw = std::atan2(R64.at<double>(1, 0), R64.at<double>(0, 0)) * 180.0 / CV_PI;
        double pitch = std::asin(-std::clamp(R64.at<double>(2, 0), -1.0, 1.0)) * 180.0 / CV_PI;
        double roll = std::atan2(R64.at<double>(2, 1), R64.at<double>(2, 2)) * 180.0 / CV_PI;
        poses.push_back({index, yaw, pitch, roll, distance});
    }
    return !poses.empty();
}