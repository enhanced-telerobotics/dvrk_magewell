#include "../include/utils.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>

// Resize image maintaining aspect ratio
cv::Mat resizeImage(const Config &config, const cv::Mat &image)
{
    if (image.empty())
    {
        return cv::Mat();
    }

    int newWidth, newHeight;
    if (config.crop)
    {
        // Keep the ratio of the image for the crop
        int h = image.rows;
        int w = image.cols;
        float scale = std::max(static_cast<float>(config.width) / w, static_cast<float>(config.height) / h);
        newWidth = static_cast<int>(w * scale);
        newHeight = static_cast<int>(h * scale);
    }
    else
    {
        // Resize the image to the target dimensions
        newWidth = config.width;
        newHeight = config.height;
    }
    
    cv::Mat resized;
    cv::resize(image, resized, cv::Size(newWidth, newHeight));
    return resized;
}

// Center crop the image to the target width
cv::Mat centerCrop(const cv::Mat &image, int targetWidth)
{
    if (image.empty())
    {
        return cv::Mat();
    }
    int w = image.cols;
    int startX = (w - targetWidth) / 2;
    startX = std::max(0, startX);
    cv::Rect cropRegion(startX, 0, std::min(targetWidth, w - startX), image.rows);
    return image(cropRegion).clone();
}

// Process command-line arguments
Config processArgs(int argc, char **argv)
{
    Config config;
    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];
        if (arg == "-d" || arg == "--disable-window-settings")
        {
            config.disableWindowSettings = true;
        }
        else if (arg == "-m" || arg == "--mono")
        {
            config.mono = true;
        }
        else if (arg == "-c" || arg == "--concat")
        {
            config.concat = true;
        }
        else if (arg == "--crop")
        {
            config.crop = true;
        }
        else if (arg == "-h" || arg == "--height")
        {
            if (i + 1 < argc)
            {
                config.height = std::stoi(argv[++i]);
            }
        }
        else if (arg == "-w" || arg == "--width")
        {
            if (i + 1 < argc)
            {
                config.width = std::stoi(argv[++i]);
            }
        }
        else if (arg == "--left-offset")
        {
            if (i + 1 < argc)
            {
                config.leftOffset = std::stoi(argv[++i]);
            }
        }
        else if (arg == "--right-offset")
        {
            if (i + 1 < argc)
            {
                config.rightOffset = std::stoi(argv[++i]);
            }
        }
    }
    return config;
}

// Set up windows
void setupWindows(const Config &config)
{
    if (config.mono)
    {
        cv::namedWindow("Left Image", cv::WINDOW_NORMAL);
        if (!config.disableWindowSettings)
        {
            cv::setWindowProperty("Left Image", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
            cv::moveWindow("Left Image", config.leftOffset, 0);
        }
    }
    else if (config.concat)
    {
        cv::namedWindow("Concatenated Image", cv::WINDOW_NORMAL);
        if (!config.disableWindowSettings)
        {
            cv::setWindowProperty("Concatenated Image", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
            cv::moveWindow("Concatenated Image", config.leftOffset, 0);
        }
    }
    else
    {
        cv::namedWindow("Left Image", cv::WINDOW_NORMAL);
        cv::namedWindow("Right Image", cv::WINDOW_NORMAL);
        if (!config.disableWindowSettings)
        {
            cv::setWindowProperty("Left Image", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
            cv::setWindowProperty("Right Image", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
            cv::moveWindow("Left Image", config.leftOffset, 0);
            cv::moveWindow("Right Image", config.rightOffset, 0);
        }
    }
}

// Display images
void displayImages(const Config &config, const cv::Mat &croppedLeft, const cv::Mat &croppedRight)
{
    if (config.mono)
        cv::imshow("Left Image", croppedLeft);
    else if (config.concat)
    {
        cv::Mat concatenated;
        cv::hconcat(croppedLeft, croppedRight, concatenated);
        cv::imshow("Concatenated Image", concatenated);
    }
    else
    {
        cv::imshow("Left Image", croppedLeft);
        cv::imshow("Right Image", croppedRight);
    }
}

// Convert an image to BGR8 format
cv::Mat to_bgr8(const sensor_msgs::msg::Image::ConstSharedPtr &msg,
                const std::string &logger_name)
{
    try
    {
        if (msg->encoding == "mono8")
        {
            auto gray = cv_bridge::toCvCopy(msg, "mono8")->image;
            cv::Mat bgr;
            cv::cvtColor(gray, bgr, cv::COLOR_GRAY2BGR);
            return bgr;
        }
        else
        {
            return cv_bridge::toCvCopy(msg, "bgr8")->image;
        }
    }
    catch (const cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger(logger_name),
                     "cv_bridge exception: %s", e.what());
        return {};
    }
}