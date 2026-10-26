#include "armor_detector.h"

bool detect_corners(
    const cv::Mat& frame,
    std::vector<std::vector<cv::Point2f>>& corners, 
    YoloVino::Yolov8poseVino& detector)
{
    corners.clear();
    if (frame.empty())
    {
        return false;
    }
    std::vector<YoloVino::NNDetectData> results = detector.safe_predict(
        frame, 
        cv::Rect(0, 0, frame.cols, frame.rows));
    std::vector<int> num;
    if (results.empty())
    {
        return false;
    }
    
    for (int i = 0; i < static_cast<int>(results.size()); i++)
    {
        bool valid = true;
        if (results[i].keypoints.size() != 4)
        {
            continue;
        }
        if (results[i].confidence < 0.5f)
        {
            continue;
        }
        for (int j = 0; j < 4; j++)
        {
            if (results[i].keypoints[j].z < 0.5f)
            {
                valid = false;
                break;
            }
        }
        if (valid)
        {
            num.push_back(i);
        }
    }
    corners.resize(num.size());
    for (int i = 0; i < static_cast<int>(num.size()); i++)
    {
        int x = num[i];
        for (int j = 0; j < 4; j++)
        {
            auto& point = results[x].keypoints[j];
            corners[i].push_back(cv::Point2f(point.x, point.y));
        }
    }
    return !corners.empty();
}