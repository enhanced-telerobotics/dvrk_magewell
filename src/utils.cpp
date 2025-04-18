#include "../include/utils.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>

// Resize image maintaining aspect ratio
cv::Mat resizeImage(const cv::Mat &image, int targetHeight, int targetWidth)
{
    int h = image.rows;
    int w = image.cols;
    float scale = std::max(static_cast<float>(targetWidth) / w, static_cast<float>(targetHeight) / h);
    int newWidth = static_cast<int>(w * scale);
    int newHeight = static_cast<int>(h * scale);
    cv::Mat resized;
    cv::resize(image, resized, cv::Size(newWidth, newHeight));
    return resized;
}

// Center crop the image to the target width
cv::Mat centerCrop(const cv::Mat &image, int targetWidth)
{
    int w = image.cols;
    int startX = (w - targetWidth) / 2;
    startX = std::max(0, startX);
    cv::Rect cropRegion(startX, 0, std::min(targetWidth, w - startX), image.rows);
    return image(cropRegion).clone();
}

// Process command-line arguments
Config processArgs(int argc, char **argv)
{
    Config config { false, false, false };
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
            cv::moveWindow("Left Image", 5120, 0);
        }
    }
    else if (config.concat)
    {
        cv::namedWindow("Concatenated Image", cv::WINDOW_NORMAL);
        if (!config.disableWindowSettings)
        {
            cv::setWindowProperty("Concatenated Image", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
            cv::moveWindow("Concatenated Image", 5120, 0);
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
            cv::moveWindow("Left Image", 5120, 0);
            cv::moveWindow("Right Image", 6144, 0);
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