#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <chrono>

using namespace std::chrono_literals;

// Initialize global variables to store the images
cv::Mat right_image;
cv::Mat left_image;

// Camera matrices and distortion coefficients for the left and right cameras
cv::Mat cameraMatrix1 = (cv::Mat_<double>(3, 3) << 1661.6079931371382, 0.0, 842.5747057704033,
                                                  0.0, 1666.1777217198353, 520.1389163730331,
                                                  0.0, 0.0, 1.0);

cv::Mat distCoeffs1 = (cv::Mat_<double>(5, 1) << -0.30752348248226674, 0.4540566708432747, 0.0093248064852139, 0.0050331457366487335, 0.5341529307552948);

cv::Mat cameraMatrix2 = (cv::Mat_<double>(3, 3) << 1655.2407247388903, 0.0, 1042.2380619837568,
                                                  0.0, 1649.76635591878, 590.2562906469209,
                                                  0.0, 0.0, 1.0);

cv::Mat distCoeffs2 = (cv::Mat_<double>(5, 1) << -0.3576372479832782, 0.35962149132853005, 0.0002005560528842616, -0.00850741051442474, 0.33326183028709666);

// Callback function to process the right image message
void right_image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        right_image = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to convert right image: %s", e.what());
    }
}

// Callback function to process the left image message
void left_image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        left_image = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to convert left image: %s", e.what());
    }
}

// Function to initialize undistort remap matrices
void init_undistort_remap(const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs, const cv::Size& image_size, 
                          cv::Mat& map1, cv::Mat& map2) {
    cv::Mat new_camera_matrix = cv::getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, image_size, 1);
    cv::initUndistortRectifyMap(camera_matrix, dist_coeffs, cv::Mat(), new_camera_matrix, image_size, CV_16SC2, map1, map2);
}

// Function to resize image while maintaining aspect ratio to both height and width constraints
cv::Mat resize_image(const cv::Mat& image, int target_height, int target_width) {
    int h = image.rows;
    int w = image.cols;
    float scaling_factor = std::max(target_width / static_cast<float>(w), target_height / static_cast<float>(h));
    int new_width = static_cast<int>(w * scaling_factor);
    int new_height = static_cast<int>(h * scaling_factor);
    cv::Mat resized_image;
    cv::resize(image, resized_image, cv::Size(new_width, new_height));
    return resized_image;
}

// Function to center-crop the image to the target width
cv::Mat center_crop(const cv::Mat& image, int target_width) {
    int w = image.cols;
    int start_x = (w - target_width) / 2;
    if (start_x < 0) {
        start_x = 0;
    }
    return image(cv::Rect(start_x, 0, target_width, image.rows));
}

// Function to display images in two separate windows
void display_images(std::shared_ptr<rclcpp::Node> node, std::string display_mode = "default") {
    // Target height and width for resized images
    int target_height = 768;
    int target_width = 1024;

    // Initialize remap matrices
    cv::Mat map1_left, map2_left;
    cv::Mat map1_right, map2_right;

    if (display_mode == "goovis") {
        target_height = 1440;
        target_width = 1280;
        // Create a window for the combined image
        cv::namedWindow("Combined Image", cv::WINDOW_NORMAL);
        // Set the window to full screen (optional)
        cv::setWindowProperty("Combined Image", cv::WND_PROP_FULLSCREEN, cv::WINDOW_NORMAL);
        // Move the window to the desired position (5120, 0)
        cv::moveWindow("Combined Image", 5120, 0);   // Right monitor (for example)
    } else {
        // Create two windows for left and right images
        cv::namedWindow("Left Image", cv::WINDOW_NORMAL);
        cv::namedWindow("Right Image", cv::WINDOW_NORMAL);

        // Set the windows to full screen (optional)
        cv::setWindowProperty("Left Image", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
        cv::setWindowProperty("Right Image", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

        // Move the windows to desired positions (5120, 0) and (0, 0)
        cv::moveWindow("Left Image", 5120, 0);   // Right monitor (for example)
        cv::moveWindow("Right Image", 6144, 0);  // Left monitor (for example)
    }

    // Initialize the FPS calculation
    auto prev_time = std::chrono::steady_clock::now();

    while (rclcpp::ok()) {
        rclcpp::spin_some(node);

        if (!right_image.empty() && !left_image.empty()) {
            // Resize both images to the target height and width
            cv::Mat left_image_resized = resize_image(left_image, target_height, target_width);
            cv::Mat right_image_resized = resize_image(right_image, target_height, target_width);

            // Initialize remap matrices if not already done
            if (map1_left.empty() && map1_right.empty()) {
                init_undistort_remap(cameraMatrix1, distCoeffs1, left_image_resized.size(), map1_left, map2_left);
                init_undistort_remap(cameraMatrix2, distCoeffs2, right_image_resized.size(), map1_right, map2_right);
            }

            // Apply remap (undistortion) to both resized images
            cv::Mat left_image_undistorted, right_image_undistorted;
            cv::remap(left_image_resized, left_image_undistorted, map1_left, map2_left, cv::INTER_LINEAR);
            cv::remap(right_image_resized, right_image_undistorted, map1_right, map2_right, cv::INTER_LINEAR);

            // Center crop the undistorted images to the target width
            cv::Mat left_image_cropped = center_crop(left_image_undistorted, target_width);
            cv::Mat right_image_cropped = center_crop(right_image_undistorted, target_width);

            // Calculate FPS
            auto curr_time = std::chrono::steady_clock::now();
            std::chrono::duration<double> elapsed = curr_time - prev_time;
            double fps = 1.0 / elapsed.count();
            prev_time = curr_time;

            // Overlay FPS on both images
            cv::putText(left_image_cropped, "LFPS: " + std::to_string(fps), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
            cv::putText(right_image_cropped, "RFPS: " + std::to_string(fps), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);

            // Display the left and right images in separate windows or side by side
            if (display_mode == "goovis") {
                // Resize images to 2560x1440
                // Concatenate images side by side without resizing
                cv::Mat combined_image;
                cv::hconcat(left_image_cropped, right_image_cropped, combined_image);

                // Display the combined image
                cv::imshow("Combined Image", combined_image);
            } else {
                // Display the left and right images in separate windows
                cv::imshow("Left Image", left_image_cropped);
                cv::imshow("Right Image", right_image_cropped);
            }

            int key = cv::waitKey(1);
            if (key == 'q') {
                break;
            }
            if (key == 'f') {
                static bool is_fullscreen = false;
                is_fullscreen = !is_fullscreen;
                cv::setWindowProperty("Combined Image", cv::WND_PROP_FULLSCREEN, is_fullscreen ? cv::WINDOW_FULLSCREEN : cv::WINDOW_NORMAL);
            }
        }
    }

    cv::destroyAllWindows();
}

int main(int argc, char **argv) {
    // Initialize rclcpp
    rclcpp::init(argc, argv);

    // Create a node
    auto node = std::make_shared<rclcpp::Node>("ros2_continuous_image_node");

    // Parse command line arguments
    std::string display_mode = "default";
    if (argc > 1 && std::string(argv[1]) == "-d" && argc > 2) {
        display_mode = argv[2];
    }

    // Subscribe to the right image topic
    auto right_image_sub = node->create_subscription<sensor_msgs::msg::Image>(
        "/davinci_endo/right/image_raw", 10, right_image_callback);

    // Subscribe to the left image topic
    auto left_image_sub = node->create_subscription<sensor_msgs::msg::Image>(
        "/davinci_endo/left/image_raw", 10, left_image_callback);

    // Run the display_images function
    try {
        display_images(node, display_mode);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(node->get_logger(), "Exception in display_images: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}
