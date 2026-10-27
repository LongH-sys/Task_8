#include "armor_detector.h"

#include "pose_solver.h"

#include "result_drawer.h"

#include "camera_manage.h"

#include "camera.h"

#include "sdk_manage.h"

#include <memory>

#include <utility>

#include <opencv2/highgui.hpp>

#include <iostream>

int main()
{
    auto logger = std::make_unique<YoloVino::YoloVinoLogger>(
        "/home/longh/vino_task/src/YoloVino/config/yolov8pose_vino_config.yaml");
    logger->print_yaml_info();
    logger->set_info_level(YoloVino::LoggerInfoLevel::warning_info);
    YoloVino::Yolov8poseVino vino(std::move(logger));
    SdkManager sdk;
    if (!sdk.initialize())
    {
        return -1;
    }
    CameraManager cam_manage;
    Camera camera(cam_manage.select_device());
    if (!camera.open())
    {
        if (camera.close())
        {
            sdk.finalize();
        }
        return -1;
    }
    if (!camera.set_trigger_mode(false))
    {
        if (camera.close())
        {
            sdk.finalize();
        }
        return -1;
    }
    if (!camera.start_grabbing())
    {
        if (camera.close())
        {
            sdk.finalize();
        }
        return -1;
    }
    cv::Mat frame;
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<PoseInfo> poses;
    cv::namedWindow("Task_8", cv::WINDOW_NORMAL);
    std::cout << "连续采集中,请按q停止采集" <<std::endl;
    while(true)
    {
        if (camera.get_frame(frame))
        {
            corners.clear();
            poses.clear();
            if (detect_corners(frame, corners, vino))
            {
                estimate_pose(corners, poses);
                draw_corners(frame, corners);
                draw_poses(frame, corners, poses);
            }
            cv::imshow("Task_8", frame);
        }
        int key = cv::waitKey(10);
        if (key == 'q' || key == 'Q' || key == 27)
        {
            break;
        }
        if (cv::getWindowProperty("Task_8", cv::WND_PROP_AUTOSIZE) < 0)
        {
            break;
        }
    }
    cv::destroyAllWindows();
    if (!camera.stop_grabbing())
    {
        if (camera.close())
        {
            sdk.finalize();
        }
        return -1;
    }
    if (!camera.close())
    {
        return -1;
    }
    if (!sdk.finalize())
    {
        return -1;
    }
    return 0;
}