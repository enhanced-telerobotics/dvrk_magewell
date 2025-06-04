#include "../include/utils.h"
#include <opencv2/opencv.hpp>
#include <iostream>

int main(int argc, char **argv)
{
    // Process command line arguments
    Config config = processArgs(argc, argv);

    cv::VideoCapture capLeft, capRight;

    if (config.device == "HD")
    {
        capLeft.open(2, cv::CAP_V4L2);
        capRight.open(0, cv::CAP_V4L2);

        capLeft.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V'));
        capLeft.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
        capLeft.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);

        capRight.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V'));
        capRight.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
        capRight.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
    }
    else if (config.device == "SD")
    {
        capLeft.open(4, cv::CAP_V4L2);
        capRight.open(5, cv::CAP_V4L2);

        capLeft.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V'));
        // capLeft.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        // capLeft.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

        capRight.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V'));
        // capRight.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        // capRight.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    }
    else
    {
        std::cerr << "Unknown device type: " << config.device << std::endl;
        return -1;
    }

    if (!capLeft.isOpened() || (!config.mono && !capRight.isOpened()))
    {
        std::cerr << "Cannot open the video cameras" << std::endl;
        return -1;
    }

    // Set up windows using the helper function
    setupWindows(config);

    while (true)
    {
        cv::Mat left_raw, right_raw;

        bool retLeft = capLeft.read(left_raw);
        bool retRight = config.mono ? true : capRight.read(right_raw);

        if (!retLeft || (!config.mono && !retRight))
        {
            std::cerr << "Failed to capture images" << std::endl;
            break;
        }

        // Apply ratio (crop or pad) to raw images
        cv::Mat left_proc, right_proc;
        if (config.method == "crop")
        {
            left_proc  = centerCrop(config, left_raw);
            right_proc = centerCrop(config, right_raw);
        }
        else if (config.method == "pad")
        {
            left_proc  = centerPadding(config, left_raw);
            right_proc = centerPadding(config, right_raw);
        }
        else
        {
            left_proc  = left_raw;
            right_proc = right_raw;
        }

        // Resize
        left_proc  = resizeImage(config, left_proc);
        right_proc = resizeImage(config, right_proc);

        // Show images according to the flags (mono / concat / stereo)
        displayImages(config, left_proc, right_proc);

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