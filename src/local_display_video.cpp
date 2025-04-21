#include "../include/utils.h"
#include <opencv2/opencv.hpp>
#include <iostream>

int main(int argc, char **argv)
{
    // Process command line arguments
    Config config = processArgs(argc, argv);

    // Open video capture devices
    cv::VideoCapture capLeft(2, cv::CAP_V4L2);
    cv::VideoCapture capRight(0, cv::CAP_V4L2);

    if (!capLeft.isOpened() || (!config.mono && !capRight.isOpened()))
    {
        std::cerr << "Cannot open the video cameras" << std::endl;
        return -1;
    }

    // Set up windows using the helper function
    setupWindows(config);

    while (true)
    {
        cv::Mat frameLeft, frameRight;

        bool retLeft = capLeft.read(frameLeft);
        bool retRight = config.mono ? true : capRight.read(frameRight);

        if (!retLeft || (!config.mono && !retRight))
        {
            std::cerr << "Failed to capture images" << std::endl;
            break;
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
            break;
        }
    }

    // Cleanup
    capLeft.release();
    if (!config.mono)
    {
        capRight.release();
    }
    cv::destroyAllWindows();

    return 0;
}