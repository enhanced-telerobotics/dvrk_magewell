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
    bool crop = false;

    int height = 1080;
    int width = 960;

    int leftOffset = 5120;
    int rightOffset = leftOffset + width;
};

// Process command-line arguments
Config processArgs(int argc, char **argv);

// Resize image maintaining aspect ratio
cv::Mat resizeImage(const Config &config, const cv::Mat &image);

// Center crop the image to the target width
cv::Mat centerCrop(const cv::Mat &image, int targetWidth);

// Set up windows
void setupWindows(const Config &config);

// Display images
void displayImages(const Config &config, const cv::Mat &croppedLeft, const cv::Mat &croppedRight);

// Convert an image to BGR8 format
cv::Mat to_bgr8(const sensor_msgs::msg::Image::ConstSharedPtr &msg,
                const std::string &logger_name);

#endif // UTILS_H