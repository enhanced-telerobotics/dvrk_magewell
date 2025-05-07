#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "../include/utils.h"

class StereoViewerNode : public rclcpp::Node
{
public:
    explicit StereoViewerNode(const Config &config)
        : Node("stereo_view"), config_(config)
    {
        // Subscribe with message_filters so the synchronizer owns the subscribers
        left_sub_.subscribe(this, "davinci_endo/left/image_raw");
        right_sub_.subscribe(this, "davinci_endo/right/image_raw");

        constexpr int queue_size = 10;
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(queue_size), left_sub_, right_sub_);
        sync_->registerCallback(std::bind(&StereoViewerNode::imageCallback, this, std::placeholders::_1, std::placeholders::_2));

        setupWindows(config_);
        RCLCPP_INFO(get_logger(), "StereoViewerNode initialised – waiting for image pairs …");
    }

    ~StereoViewerNode() override
    {
        cv::destroyAllWindows();
    }

private:
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
                                                                       sensor_msgs::msg::Image>;

    // -----------------------------  Callback  -----------------------------------------
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &left_msg,
                       const sensor_msgs::msg::Image::ConstSharedPtr &right_msg)
    {
        // Convert ROS messages → OpenCV BGR8
        cv::Mat left_raw  = to_bgr8(left_msg,  this->get_name());
        cv::Mat right_raw = to_bgr8(right_msg, this->get_name());
        if (left_raw.empty() || right_raw.empty())
            return;  // conversion failed

        // Resize (and optionally overscale for cropping)
        cv::Mat left_proc  = resizeImage(config_, left_raw);
        cv::Mat right_proc = resizeImage(config_, right_raw);

        // Optional centre‑crop to requested width
        if (config_.crop)
        {
            left_proc  = centerCrop(left_proc,  config_.width);
            right_proc = centerCrop(right_proc, config_.width);
        }

        // Show images according to the flags (mono / concat / stereo)
        displayImages(config_, left_proc, right_proc);

        // HighGUI needs a brief wait to refresh
        cv::waitKey(1);
    }

    // -----------------------------  Members  ------------------------------------------
    Config config_;
    message_filters::Subscriber<sensor_msgs::msg::Image> left_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> right_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    Config cfg = processArgs(argc, argv);
    auto node = std::make_shared<StereoViewerNode>(cfg);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
