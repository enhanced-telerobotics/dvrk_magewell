#include "../include/utils.h"
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <iostream>
#include <mutex>
#include <condition_variable>

#define DISPLAY_HEIGHT 768
#define DISPLAY_WIDTH 1024

std::mutex leftMutex, rightMutex;
cv::Mat leftFrame, rightFrame;
bool leftFrameReady = false, rightFrameReady = false;

void leftImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(leftMutex);
    try
    {
        leftFrame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        leftFrameReady = true;
    }
    catch (cv_bridge::Exception &e)
    {
        std::cerr << "cv_bridge exception (left): " << e.what() << std::endl;
    }
}

void rightImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(rightMutex);
    try
    {
        rightFrame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        rightFrameReady = true;
    }
    catch (cv_bridge::Exception &e)
    {
        std::cerr << "cv_bridge exception (right): " << e.what() << std::endl;
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
            std::cerr << "Waiting for frames..." << std::endl;
            loopRate.sleep();
            continue;
        }

        // Resize and center crop
        cv::Mat resizedLeft = resizeImage(frameLeft, DISPLAY_HEIGHT, DISPLAY_WIDTH);
        cv::Mat croppedLeft = centerCrop(resizedLeft, DISPLAY_WIDTH);
        cv::Mat croppedRight;

        if (!config.mono)
        {
            cv::Mat resizedRight = resizeImage(frameRight, DISPLAY_HEIGHT, DISPLAY_WIDTH);
            croppedRight = centerCrop(resizedRight, DISPLAY_WIDTH);
        }

        // Display images through the helper function
        displayImages(config, croppedLeft, croppedRight);

        if (cv::waitKey(1) == 'q')
        {
            break;
        }

        loopRate.sleep();
    }

    // Cleanup
    cv::destroyAllWindows();
    rclcpp::shutdown();

    return 0;
}