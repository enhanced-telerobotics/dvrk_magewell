#ifndef UTILS_H
#define UTILS_H

#include <opencv2/opencv.hpp>
#include <string>

// Resize image maintaining aspect ratio
cv::Mat resizeImage(const cv::Mat &image, int targetHeight, int targetWidth);

// Center crop the image to the target width
cv::Mat centerCrop(const cv::Mat &image, int targetWidth);

// Struct to hold configuration
struct Config {
    bool disableWindowSettings;
    bool mono;
    bool concat;
};

// Process command-line arguments
Config processArgs(int argc, char **argv);

// Set up windows
void setupWindows(const Config &config);

// Display images
void displayImages(const Config &config, const cv::Mat &croppedLeft, const cv::Mat &croppedRight);

#endif // UTILS_H