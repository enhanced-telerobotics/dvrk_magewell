#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

// Function to publish images from a camera
void publish_image(const cv::Mat &frame, image_transport::Publisher &publisher, const std::shared_ptr<rclcpp::Node> &node)
{
    if (!frame.empty())
    {
        // Convert the frame to a ROS2 message and publish
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        msg->header.stamp = node->now(); // Set time stamp
        publisher.publish(msg);
    }
}

int main(int argc, char **argv)
{
    // Initialize ROS2
    rclcpp::init(argc, argv);

    // Create the node
    auto node = rclcpp::Node::make_shared("publish_video");

    // Image transport setup
    image_transport::ImageTransport it(node);
    image_transport::Publisher pub_left = it.advertise("davinci_endo/left/image_raw", 10);
    image_transport::Publisher pub_right = it.advertise("davinci_endo/right/image_raw", 10);

    // Open video capture devices
    // Note: Run `v4l2-ctl --list-devices` to inspect camera devices
    cv::VideoCapture cap_left(2, cv::CAP_V4L2);
    cv::VideoCapture cap_right(0, cv::CAP_V4L2);

    if (!cap_left.isOpened() || !cap_right.isOpened())
    {
        RCLCPP_ERROR(node->get_logger(), "Cannot open the video cameras");
        return -1;
    }

    cv::Mat frame_left, frame_right;
    rclcpp::Rate loop_rate(120); // Loop maximum frequency

    bool running = true; // Boolean flag to alternate messages

    // Main loop
    while (rclcpp::ok())
    {
        // Capture frames from both cameras
        cap_left >> frame_left;
        cap_right >> frame_right;

        // Publish frames from both cameras
        publish_image(frame_left, pub_left, node);
        publish_image(frame_right, pub_right, node);

        // Alternate messages every 3 seconds
        static auto last_message_time = node->now();
        auto current_time = node->now();
        if ((current_time - last_message_time).seconds() >= 3.0) // Alternate message every 3 seconds
        {
            std::cout << "\r" << (running ? "Running..." : "Active...") << std::flush;
            running = !running;               // Toggle the message
            last_message_time = current_time; // Update last message time
        }

        // Handle callbacks and sleep
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    // Shutdown ROS2
    rclcpp::shutdown();
    return 0;
}
