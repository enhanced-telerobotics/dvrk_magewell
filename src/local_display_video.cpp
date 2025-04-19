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
        cv::Mat resizedLeft = resizeImage(frameLeft, config.height, config.width);
        cv::Mat resizedRight = resizeImage(frameRight, config.height, config.width);

        if (config.crop)
        {
            cv::Mat resizedLeft = centerCrop(resizedLeft, config.width);
            cv::Mat resizedRight = centerCrop(resizedRight, config.width);
        }

        // Display images through the helper function
        displayImages(config, resizedLeft, resizedRight);

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