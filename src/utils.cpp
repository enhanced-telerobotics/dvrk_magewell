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

    int targetWidth = config.width;
    int targetHeight = config.height;
    
    cv::Mat resized;
    cv::resize(image, resized, cv::Size(targetWidth, targetHeight));
    return resized;
}

// Center crop the image to the target aspect ratio only
cv::Mat centerCrop(const Config &config, const cv::Mat &image)
{
    if (image.empty())
    {
        return cv::Mat();
    }

    int w = image.cols;
    int h = image.rows;
    float targetRatio = config.ratio;
    float imageRatio = static_cast<float>(w) / h;

    int cropW, cropH, startX, startY;
    if (imageRatio > targetRatio) {
        // Image is wider than target: crop width
        cropH = h;
        cropW = static_cast<int>(h * targetRatio);
        startX = (w - cropW) / 2;
        startY = 0;
    } else {
        // Image is taller than target: crop height
        cropW = w;
        cropH = static_cast<int>(w / targetRatio);
        startX = 0;
        startY = (h - cropH) / 2;
    }
    cv::Rect cropRegion(startX, startY, cropW, cropH);
    return image(cropRegion).clone();
}

// Center pad the image to the target aspect ratio only (with black borders)
cv::Mat centerPadding(const Config &config, const cv::Mat &image)
{
    if (image.empty())
    {
        return cv::Mat();
    }

    int w = image.cols;
    int h = image.rows;
    float targetRatio = config.ratio;
    float imageRatio = static_cast<float>(w) / h;

    int newW, newH;
    if (imageRatio > targetRatio) {
        // Image is wider than target: pad height
        newW = w;
        newH = static_cast<int>(w / targetRatio);
    } else {
        // Image is taller than target: pad width
        newH = h;
        newW = static_cast<int>(h * targetRatio);
    }
    int top = (newH - h) / 2;
    int bottom = newH - h - top;
    int left = (newW - w) / 2;
    int right = newW - w - left;

    cv::Mat padded;
    cv::copyMakeBorder(image, padded, top, bottom, left, right, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
    return padded;
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
        else if (arg == "--method")
        {
            if (i + 1 < argc)
            {
                config.method = argv[++i]; // Accept string, not float
            }
        }
        else if (arg == "--device")
        {
            if (i + 1 < argc)
            {
                config.device = argv[++i];
            }
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
        else if (arg == "--ratio")
        {
            if (i + 1 < argc)
            {
                std::string ratioStr = argv[++i];
                size_t colonPos = ratioStr.find(':');
                if (colonPos != std::string::npos)
                {
                    float num = std::stof(ratioStr.substr(0, colonPos));
                    float denom = std::stof(ratioStr.substr(colonPos + 1));
                    if (denom != 0.0f)
                    {
                        config.ratio = num / denom;
                    }
                }
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