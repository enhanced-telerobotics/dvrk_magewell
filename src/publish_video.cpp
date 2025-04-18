#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <string>

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

    // Default parameters
    std::string topic_name = "davinci_endo/default/image_raw";
    int device_id = 0;

    // Simple argument parsing for --topic and --device
    for (int i = 1; i < argc; i++)
    {
        if (std::string(argv[i]) == "--topic" && i + 1 < argc)
        {
            topic_name = argv[++i];
        }
        else if (std::string(argv[i]) == "--device" && i + 1 < argc)
        {
            device_id = std::stoi(argv[++i]);
        }
    }

    // Image transport setup
    image_transport::ImageTransport it(node);
    image_transport::Publisher publisher = it.advertise(topic_name, 10);

    // Open video capture device using the provided device id
    cv::VideoCapture cap(device_id, cv::CAP_V4L2);

    if (!cap.isOpened())
    {
        RCLCPP_ERROR(node->get_logger(), "Cannot open the video camera with device id %d", device_id);
        return -1;
    }

    cv::Mat frame;
    // Loop maximum frequency, slightly higher than 60 FPS
    rclcpp::Rate loop_rate(65);

    // Main loop
    while (rclcpp::ok())
    {
        // Capture frame from the camera
        if (cap.read(frame))
        {
            // Publish frame from the camera
            publish_image(frame, publisher, node);
        }

        // Handle callbacks and sleep
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    // Shutdown ROS2
    rclcpp::shutdown();
    return 0;
}
