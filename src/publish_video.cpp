#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <string>
#include <fstream>

// Function to load camera info from YAML file
bool loadCameraInfo(const std::string &yaml_file, sensor_msgs::msg::CameraInfo &camera_info, const std::shared_ptr<rclcpp::Node> &node)
{
    try
    {
        // Load YAML file
        YAML::Node config = YAML::LoadFile(yaml_file);

        camera_info.width = config["image_width"].as<uint32_t>();
        camera_info.height = config["image_height"].as<uint32_t>();

        // Camera name
        if (config["camera_name"])
        {
            camera_info.header.frame_id = config["camera_name"].as<std::string>();
        }

        // Distortion model
        camera_info.distortion_model = config["distortion_model"].as<std::string>();

        // Camera matrix (K)
        auto K_data = config["camera_matrix"]["data"].as<std::vector<double>>();
        if (K_data.size() != 9)
        {
            RCLCPP_ERROR(node->get_logger(), "Invalid camera_matrix size");
            return false;
        }
        std::copy(K_data.begin(), K_data.end(), camera_info.k.begin());

        // Distortion coefficients (D)
        auto D_data = config["distortion_coefficients"]["data"].as<std::vector<double>>();
        camera_info.d = D_data;

        // Rectification matrix (R)
        auto R_data = config["rectification_matrix"]["data"].as<std::vector<double>>();
        if (R_data.size() != 9)
        {
            RCLCPP_ERROR(node->get_logger(), "Invalid rectification_matrix size");
            return false;
        }
        std::copy(R_data.begin(), R_data.end(), camera_info.r.begin());

        // Projection matrix (P)
        auto P_data = config["projection_matrix"]["data"].as<std::vector<double>>();
        if (P_data.size() != 12)
        {
            RCLCPP_ERROR(node->get_logger(), "Invalid projection_matrix size");
            return false;
        }
        std::copy(P_data.begin(), P_data.end(), camera_info.p.begin());

        RCLCPP_INFO(node->get_logger(), "Successfully loaded camera info from %s", yaml_file.c_str());
        RCLCPP_INFO(node->get_logger(), "Image size: %dx%d", camera_info.width, camera_info.height);
        RCLCPP_INFO(node->get_logger(), "Distortion model: %s", camera_info.distortion_model.c_str());

        return true;
    }
    catch (const YAML::Exception &e)
    {
        RCLCPP_ERROR(node->get_logger(), "YAML parsing error: %s", e.what());
        return false;
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node->get_logger(), "Error loading camera info: %s", e.what());
        return false;
    }
}

// Function to publish images from a camera
void publish_image(const cv::Mat &frame, image_transport::Publisher &publisher, 
                   rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub,
                   sensor_msgs::msg::CameraInfo &camera_info,
                   const std::shared_ptr<rclcpp::Node> &node)
{
    if (!frame.empty())
    {
        // Set timestamp for synchronization
        auto timestamp = node->now();

        // Convert the frame to a ROS2 message and publish
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        msg->header.stamp = timestamp;
        publisher.publish(msg);

        // Publish camera info if available
        if (camera_info_pub != nullptr)
        {
            camera_info.header.stamp = timestamp;
            camera_info_pub->publish(camera_info);
        }
    }
}

int main(int argc, char **argv)
{
    // Initialize ROS2
    rclcpp::init(argc, argv);

    // Create the node
    auto node = rclcpp::Node::make_shared("publish_video");

    // Default parameters
    std::string topic_name = "image_raw";
    std::string camera_info_topic_name = "camera_info";
    int device_id = 0;
    int target_height = 0; // Default: no resizing
    std::string yaml_file = "";
    std::string frame_id = "camera";

    // Parse ROS2 parameters for topic, device, height, and camera info
    node->declare_parameter<std::string>("topic", topic_name);
    node->declare_parameter<std::string>("camera_info_topic", camera_info_topic_name);
    node->declare_parameter<int>("device", device_id);
    node->declare_parameter<int>("height", target_height);
    node->declare_parameter<std::string>("yaml_file", yaml_file);
    node->declare_parameter<std::string>("frame_id", frame_id);

    node->get_parameter("topic", topic_name);
    node->get_parameter("camera_info_topic", camera_info_topic_name);
    node->get_parameter("device", device_id);
    node->get_parameter("height", target_height);
    node->get_parameter("yaml_file", yaml_file);
    node->get_parameter("frame_id", frame_id);

    // Image transport setup
    image_transport::ImageTransport it(node);
    image_transport::Publisher publisher = it.advertise(topic_name, 10);

    // Camera info setup (optional)
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub = nullptr;
    sensor_msgs::msg::CameraInfo camera_info;
    
    if (!yaml_file.empty())
    {
        // Load camera info from YAML file
        if (loadCameraInfo(yaml_file, camera_info, node))
        {
            // Override frame_id if provided as parameter
            if (!frame_id.empty())
            {
                camera_info.header.frame_id = frame_id;
            }
            
            // Create camera info publisher
            camera_info_pub = node->create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic_name, 10);
            RCLCPP_INFO(node->get_logger(), "Camera info will be published to: %s", camera_info_topic_name.c_str());
        }
        else
        {
            RCLCPP_WARN(node->get_logger(), "Failed to load camera info from %s, continuing without camera info", yaml_file.c_str());
        }
    }

    // Open video capture device using the provided device id
    cv::VideoCapture cap(device_id, cv::CAP_V4L2);
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V'));

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
            // Resize frame if target height is specified
            if (target_height > 0)
            {
                int original_height = frame.rows;
                int original_width = frame.cols;
                int target_width = (target_height * original_width) / original_height;
                cv::resize(frame, frame, cv::Size(target_width, target_height));
            }

            // Publish frame from the camera (and camera info if available)
            publish_image(frame, publisher, camera_info_pub, camera_info, node);
        }

        // Handle callbacks and sleep
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    // Shutdown ROS2
    rclcpp::shutdown();
    return 0;
}
