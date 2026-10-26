#include "result_drawer.h"

#include <opencv2/imgproc.hpp>

void draw_corners(
    cv::Mat& frame, 
    const std::vector<std::vector<cv::Point2f>>& corners)
{
    if (frame.empty() || corners.empty())
    {
        return;
    }
    for (int i = 0; i < static_cast<int>(corners.size()); i++)
    {
        for (int j = 0; j < 4; j++)
        {
            cv::circle(frame, corners[i][j], 5, cv::Scalar(0, 255, 0), -1);
        }
    }
}

void draw_poses(
    cv::Mat& frame, 
    const std::vector<std::vector<cv::Point2f>>& corners,
    const std::vector<PoseInfo>& poses)
{
    if (frame.empty() || poses.empty())
    {
        return;
    }
    for (int i = 0; i < static_cast<int>(poses.size()); i++)
    {
        int index = poses[i].index;
        float x = corners[index][3].x + 10.0f;
        float y = corners[index][3].y + 20.0f;
        cv::Point2f org(x, y);
        cv::Point2f distance_org(x, y + 20.0f);
        cv::putText(
            frame,
            cv::format("Yaw: %.2f  Pitch: %.2f  Roll: %.2f",
                poses[i].yaw_deg, poses[i].pitch_deg, poses[i].roll_deg),
            org,
            cv::FONT_HERSHEY_SIMPLEX,
            0.5,
            cv::Scalar(0, 255, 0),
            1);
        cv::putText(
            frame,
            cv::format("Distance: %.2f", poses[i].distance_mm),
            distance_org,
            cv::FONT_HERSHEY_SIMPLEX,
            0.5,
            cv::Scalar(0, 255, 0),
            1);
    }
}