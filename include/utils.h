#ifndef UTILS_H
#define UTILS_H

#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>

// Struct to hold configuration
struct Config
{
    bool disableWindowSettings = false;
    bool mono = false;
    bool concat = false;

    std::string method = "original";
    std::string device = "HD";

    int height = 1080;
    int width = 960;
    float ratio = height / static_cast<float>(width);

    int leftOffset = 5120;
    int rightOffset = leftOffset + width;
};

// Process command-line arguments
Config processArgs(int argc, char **argv);

// Resize image maintaining aspect ratio
cv::Mat resizeImage(const Config &config, const cv::Mat &image);

// Center crop the image to the target width and height
cv::Mat centerCrop(const Config &config, const cv::Mat &image);

// Center pad the image to the target width and height with black borders
cv::Mat centerPadding(const Config &config, const cv::Mat &image);

// Set up windows
void setupWindows(const Config &config);

// Display images
void displayImages(const Config &config, const cv::Mat &croppedLeft, const cv::Mat &croppedRight);

// Convert an image to BGR8 format
cv::Mat to_bgr8(const sensor_msgs::msg::Image::ConstSharedPtr &msg,
                const std::string &logger_name);

#endif // UTILS_H