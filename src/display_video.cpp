#include "../include/utils.h"
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <iostream>
#include <mutex>
#include <condition_variable>

std::mutex leftMutex, rightMutex;
cv::Mat leftFrame, rightFrame;
bool leftFrameReady = false, rightFrameReady = false;

void leftImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(leftMutex);
    leftFrame = to_bgr8(msg, "video_display_node");
    if (!leftFrame.empty())
    {
        leftFrameReady = true;
    }
}

void rightImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(rightMutex);
    rightFrame = to_bgr8(msg, "video_display_node");
    if (!rightFrame.empty())
    {
        rightFrameReady = true;
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("video_display_node");

    // Process command line arguments
    Config config = processArgs(argc, argv);

    // Set up ROS 2 subscriptions
    auto leftSub = node->create_subscription<sensor_msgs::msg::Image>(
        "davinci_endo/left/image_raw", 10, leftImageCallback);

    auto rightSub = config.mono ? nullptr : node->create_subscription<sensor_msgs::msg::Image>(
        "davinci_endo/right/image_raw", 10, rightImageCallback);

    // Set up windows
    setupWindows(config);

    rclcpp::Rate loopRate(60);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);

        cv::Mat frameLeft, frameRight;

        {
            std::lock_guard<std::mutex> lock(leftMutex);
            if (leftFrameReady)
            {
                frameLeft = leftFrame.clone();
                leftFrameReady = false;
            }
        }

        if (!config.mono)
        {
            std::lock_guard<std::mutex> lock(rightMutex);
            if (rightFrameReady)
            {
                frameRight = rightFrame.clone();
                rightFrameReady = false;
            }
        }

        if (frameLeft.empty() || (!config.mono && frameRight.empty()))
        {
            RCLCPP_WARN(rclcpp::get_logger("video_display_node"), "Waiting for frames...");
            loopRate.sleep();
            continue;
        }

        // Resize and center crop
        cv::Mat resizedLeft = resizeImage(config, frameLeft);
        cv::Mat resizedRight = resizeImage(config, frameRight);

        cv::Mat croppedLeft, croppedRight;
        if (config.crop)
        {
            croppedLeft = centerCrop(resizedLeft, config.width);
            croppedRight = centerCrop(resizedRight, config.width);
        }
        else
        {
            croppedLeft = resizedLeft;
            croppedRight = resizedRight;
        }

        // Display images through the helper function
        displayImages(config, croppedLeft, croppedRight);

        if (cv::waitKey(1) == 'q')
        {
            RCLCPP_INFO(rclcpp::get_logger("video_display_node"), "Exiting...");
            break;
        }

        loopRate.sleep();
    }

    // Cleanup
    cv::destroyAllWindows();
    rclcpp::shutdown();

    return 0;
}