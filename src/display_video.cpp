#include <SDL2/SDL.h>

#include <image_transport/subscriber_filter.hpp>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

struct Config
{
    bool disable_window_settings = false;
    bool mono = false;
    bool concat = false;
    std::string method = "original";
    int height = 1080;
    int width = 1920;
    double ratio = 0.0;  // A value of zero means width / height.
    int left_offset = 5120;
    int right_offset = 0;
};

Config processArgs(int argc, char **argv)
{
    Config config;
    bool right_offset_was_set = false;

    for (int i = 1; i < argc; ++i)
    {
        const std::string arg(argv[i]);
        auto nextValue = [&]() -> std::string {
            if (i + 1 >= argc)
            {
                throw std::invalid_argument("Missing value after " + arg);
            }
            return argv[++i];
        };

        if (arg == "-d" || arg == "--disable-window-settings")
        {
            config.disable_window_settings = true;
        }
        else if (arg == "-m" || arg == "--mono")
        {
            config.mono = true;
        }
        else if (arg == "-c" || arg == "--concat")
        {
            config.concat = true;
        }
        else if (arg == "--method")
        {
            config.method = nextValue();
        }
        else if (arg == "--crop")
        {
            config.method = "crop";
        }
        else if (arg == "--pad")
        {
            config.method = "pad";
        }
        else if (arg == "--device")
        {
            (void)nextValue();  // Retained for command-line compatibility.
        }
        else if (arg == "-h" || arg == "--height")
        {
            config.height = std::stoi(nextValue());
        }
        else if (arg == "-w" || arg == "--width")
        {
            config.width = std::stoi(nextValue());
        }
        else if (arg == "--ratio")
        {
            const std::string value = nextValue();
            const std::size_t colon = value.find(':');
            if (colon == std::string::npos)
            {
                throw std::invalid_argument("Ratio must have the form width:height");
            }
            const double numerator = std::stod(value.substr(0, colon));
            const double denominator = std::stod(value.substr(colon + 1));
            if (numerator <= 0.0 || denominator <= 0.0)
            {
                throw std::invalid_argument("Ratio components must be positive");
            }
            config.ratio = numerator / denominator;
        }
        else if (arg == "--left-offset")
        {
            config.left_offset = std::stoi(nextValue());
        }
        else if (arg == "--right-offset")
        {
            config.right_offset = std::stoi(nextValue());
            right_offset_was_set = true;
        }
    }

    if (config.width <= 0 || config.height <= 0)
    {
        throw std::invalid_argument("Display width and height must be positive");
    }
    if (config.method != "original" && config.method != "crop" && config.method != "pad")
    {
        throw std::invalid_argument("--method must be original, crop, or pad");
    }
    if (!right_offset_was_set)
    {
        config.right_offset = config.left_offset + config.width;
    }
    if (config.ratio == 0.0)
    {
        config.ratio = static_cast<double>(config.width) / config.height;
    }
    return config;
}

struct PixelView
{
    const void *pixels = nullptr;
    int pitch = 0;
    Uint32 format = SDL_PIXELFORMAT_UNKNOWN;
};

struct TextureState
{
    SDL_Texture *texture = nullptr;
    int width = 0;
    int height = 0;
    Uint32 format = SDL_PIXELFORMAT_UNKNOWN;
};

struct View
{
    SDL_Window *window = nullptr;
    SDL_Renderer *renderer = nullptr;
    int logical_width = 0;
    int logical_height = 0;
};

class StereoViewerNode : public rclcpp::Node
{
public:
    explicit StereoViewerNode(const Config &config)
        : Node("stereo_view_sdl"), config_(config)
    {
        try
        {
            setupViews();
        }
        catch (...)
        {
            destroyViews();
            throw;
        }

        const auto transport = declare_parameter<std::string>("image_transport", "raw");
        profile_ = declare_parameter<bool>("profile", false);
        const int queue_size = declare_parameter<int>("sync_queue_size", 10);
        const double lower_bound_ms =
            declare_parameter<double>("sync_inter_message_lower_bound_ms", 0.0);
        if (queue_size < 1)
        {
            throw std::invalid_argument("sync_queue_size must be positive");
        }
        if (lower_bound_ms < 0.0)
        {
            throw std::invalid_argument(
                "sync_inter_message_lower_bound_ms must be non-negative");
        }
        // Resolve base-topic remappings before the transport appends /compressed.
        left_sub_.subscribe(this, get_node_topics_interface()->resolve_topic_name(
            "davinci_endo/left/image_raw"), transport, rmw_qos_profile_sensor_data);
        right_sub_.subscribe(this, get_node_topics_interface()->resolve_topic_name(
            "davinci_endo/right/image_raw"), transport, rmw_qos_profile_sensor_data);
        RCLCPP_INFO(get_logger(), "Image transport: %s", transport.c_str());

        SyncPolicy sync_policy(queue_size);
        if (lower_bound_ms > 0.0)
        {
            const auto lower_bound =
                rclcpp::Duration::from_seconds(lower_bound_ms / 1000.0);
            sync_policy.setInterMessageLowerBound(0, lower_bound);
            sync_policy.setInterMessageLowerBound(1, lower_bound);
        }
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            static_cast<const SyncPolicy &>(sync_policy), left_sub_, right_sub_);
        sync_->registerCallback(std::bind(
            &StereoViewerNode::imageCallback, this,
            std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(
            get_logger(),
            "SDL stereo viewer initialized (%dx%d per eye, mode=%s, method=%s)",
            config_.width, config_.height,
            config_.mono ? "mono" : (config_.concat ? "side-by-side" : "two-window"),
            config_.method.c_str());
    }

    ~StereoViewerNode() override
    {
        destroyTexture(left_texture_);
        destroyTexture(right_texture_);
        destroyViews();
    }

    bool processEvents()
    {
        SDL_Event event;
        while (SDL_PollEvent(&event) != 0)
        {
            if (event.type == SDL_QUIT)
            {
                return false;
            }
            if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_ESCAPE)
            {
                return false;
            }
            if (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_CLOSE)
            {
                return false;
            }
        }
        return true;
    }

private:
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image>;

    static void destroyTexture(TextureState &state)
    {
        if (state.texture != nullptr)
        {
            SDL_DestroyTexture(state.texture);
        }
        state = TextureState{};
    }

    static void destroyView(View &view)
    {
        if (view.renderer != nullptr)
        {
            SDL_DestroyRenderer(view.renderer);
        }
        if (view.window != nullptr)
        {
            SDL_DestroyWindow(view.window);
        }
        view = View{};
    }

    void destroyViews()
    {
        destroyView(right_view_);
        destroyView(main_view_);
    }

    View createView(const char *title, int width, int height, int x_offset)
    {
        View view;
        view.logical_width = width;
        view.logical_height = height;
        view.window = SDL_CreateWindow(
            title, SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
            width, height, SDL_WINDOW_SHOWN);
        if (view.window == nullptr)
        {
            throw std::runtime_error(std::string("SDL_CreateWindow failed: ") + SDL_GetError());
        }

        if (!config_.disable_window_settings)
        {
            SDL_SetWindowPosition(view.window, x_offset, 0);
            if (SDL_SetWindowFullscreen(view.window, SDL_WINDOW_FULLSCREEN_DESKTOP) != 0)
            {
                const std::string error = SDL_GetError();
                SDL_DestroyWindow(view.window);
                throw std::runtime_error("SDL_SetWindowFullscreen failed: " + error);
            }
        }

        view.renderer = SDL_CreateRenderer(view.window, -1, SDL_RENDERER_ACCELERATED);
        if (view.renderer == nullptr)
        {
            // Software rendering is useful with remote desktops and some test systems.
            view.renderer = SDL_CreateRenderer(view.window, -1, SDL_RENDERER_SOFTWARE);
        }
        if (view.renderer == nullptr)
        {
            const std::string error = SDL_GetError();
            SDL_DestroyWindow(view.window);
            throw std::runtime_error("SDL_CreateRenderer failed: " + error);
        }
        if (SDL_RenderSetLogicalSize(view.renderer, width, height) != 0)
        {
            const std::string error = SDL_GetError();
            SDL_DestroyRenderer(view.renderer);
            SDL_DestroyWindow(view.window);
            throw std::runtime_error("SDL_RenderSetLogicalSize failed: " + error);
        }
        SDL_SetRenderDrawColor(view.renderer, 0, 0, 0, 255);
        SDL_RendererInfo renderer_info;
        if (SDL_GetRendererInfo(view.renderer, &renderer_info) == 0)
        {
            RCLCPP_INFO(
                get_logger(), "SDL renderer: %s (accelerated=%s, vsync=%s)",
                renderer_info.name,
                (renderer_info.flags & SDL_RENDERER_ACCELERATED) ? "yes" : "no",
                (renderer_info.flags & SDL_RENDERER_PRESENTVSYNC) ? "yes" : "no");
        }
        return view;
    }

    void setupViews()
    {
        SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "linear");
        if (config_.mono)
        {
            main_view_ = createView("Left Image (SDL)", config_.width, config_.height,
                                    config_.left_offset);
        }
        else if (config_.concat)
        {
            main_view_ = createView("Side by Side Preview (SDL)", config_.width * 2,
                                    config_.height, config_.left_offset);
        }
        else
        {
            main_view_ = createView("Left Image (SDL)", config_.width, config_.height,
                                    config_.left_offset);
            right_view_ = createView("Right Image (SDL)", config_.width, config_.height,
                                     config_.right_offset);
        }
    }

    bool makePixelView(const sensor_msgs::msg::Image &message,
                       std::vector<std::uint8_t> &scratch, PixelView &view)
    {
        int bytes_per_pixel = 0;
        if (message.encoding == "bgr8" || message.encoding == "8UC3")
        {
            view.format = SDL_PIXELFORMAT_BGR24;
            bytes_per_pixel = 3;
        }
        else if (message.encoding == "rgb8")
        {
            view.format = SDL_PIXELFORMAT_RGB24;
            bytes_per_pixel = 3;
        }
        else if (message.encoding == "bgra8")
        {
            view.format = SDL_PIXELFORMAT_BGRA32;
            bytes_per_pixel = 4;
        }
        else if (message.encoding == "rgba8")
        {
            view.format = SDL_PIXELFORMAT_RGBA32;
            bytes_per_pixel = 4;
        }
        else if (message.encoding == "yuv422")
        {
            view.format = SDL_PIXELFORMAT_UYVY;
            bytes_per_pixel = 2;
        }
        else if (message.encoding == "yuv422_yuy2")
        {
            view.format = SDL_PIXELFORMAT_YUY2;
            bytes_per_pixel = 2;
        }
        else if (message.encoding == "mono8" || message.encoding == "8UC1")
        {
            bytes_per_pixel = 1;
            if (!validateImage(message, bytes_per_pixel))
            {
                return false;
            }
            scratch.resize(static_cast<std::size_t>(message.width) * message.height * 3U);
            for (std::uint32_t y = 0; y < message.height; ++y)
            {
                const std::uint8_t *source = message.data.data() + y * message.step;
                std::uint8_t *destination = scratch.data() +
                    static_cast<std::size_t>(y) * message.width * 3U;
                for (std::uint32_t x = 0; x < message.width; ++x)
                {
                    destination[x * 3U] = source[x];
                    destination[x * 3U + 1U] = source[x];
                    destination[x * 3U + 2U] = source[x];
                }
            }
            view.pixels = scratch.data();
            view.pitch = static_cast<int>(message.width * 3U);
            view.format = SDL_PIXELFORMAT_RGB24;
            return true;
        }
        else
        {
            if (reported_encodings_.insert(message.encoding).second)
            {
                RCLCPP_ERROR(get_logger(), "Unsupported image encoding: %s",
                             message.encoding.c_str());
            }
            return false;
        }

        if (!validateImage(message, bytes_per_pixel))
        {
            return false;
        }
        view.pixels = message.data.data();
        view.pitch = static_cast<int>(message.step);
        return true;
    }

    bool validateImage(const sensor_msgs::msg::Image &message, int bytes_per_pixel)
    {
        const std::size_t minimum_step =
            static_cast<std::size_t>(message.width) * bytes_per_pixel;
        const std::size_t required_size =
            static_cast<std::size_t>(message.step) * message.height;
        if (message.width == 0 || message.height == 0 ||
            message.step < minimum_step || message.data.size() < required_size)
        {
            RCLCPP_ERROR(get_logger(),
                         "Malformed %s image (%ux%u, step=%u, data=%zu)",
                         message.encoding.c_str(), message.width, message.height,
                         message.step, message.data.size());
            return false;
        }
        return true;
    }

    bool uploadImage(SDL_Renderer *renderer, TextureState &state,
                     const sensor_msgs::msg::Image &message,
                     std::vector<std::uint8_t> &scratch)
    {
        PixelView pixels;
        if (!makePixelView(message, scratch, pixels))
        {
            return false;
        }

        const int width = static_cast<int>(message.width);
        const int height = static_cast<int>(message.height);
        if (state.texture == nullptr || state.width != width ||
            state.height != height || state.format != pixels.format)
        {
            destroyTexture(state);
            state.texture = SDL_CreateTexture(
                renderer, pixels.format, SDL_TEXTUREACCESS_STREAMING, width, height);
            if (state.texture == nullptr)
            {
                RCLCPP_ERROR(get_logger(), "SDL_CreateTexture failed: %s", SDL_GetError());
                return false;
            }
            state.width = width;
            state.height = height;
            state.format = pixels.format;
        }

        if (SDL_UpdateTexture(state.texture, nullptr, pixels.pixels, pixels.pitch) != 0)
        {
            RCLCPP_ERROR(get_logger(), "SDL_UpdateTexture failed: %s", SDL_GetError());
            return false;
        }
        return true;
    }

    void calculateRects(const TextureState &texture, const SDL_Rect &eye,
                        SDL_Rect &source, SDL_Rect &destination) const
    {
        source = SDL_Rect{0, 0, texture.width, texture.height};
        destination = eye;
        const double image_ratio = static_cast<double>(texture.width) / texture.height;

        if (config_.method == "crop")
        {
            if (image_ratio > config_.ratio)
            {
                source.w = std::max(1, static_cast<int>(texture.height * config_.ratio));
                source.x = (texture.width - source.w) / 2;
            }
            else
            {
                source.h = std::max(1, static_cast<int>(texture.width / config_.ratio));
                source.y = (texture.height - source.h) / 2;
            }
        }
        else if (config_.method == "pad")
        {
            if (image_ratio > config_.ratio)
            {
                destination.h = std::max(
                    1, static_cast<int>(eye.h * config_.ratio / image_ratio));
                destination.y += (eye.h - destination.h) / 2;
            }
            else
            {
                destination.w = std::max(
                    1, static_cast<int>(eye.w * image_ratio / config_.ratio));
                destination.x += (eye.w - destination.w) / 2;
            }
        }
    }

    void copyTexture(SDL_Renderer *renderer, const TextureState &texture,
                     const SDL_Rect &eye)
    {
        SDL_Rect source;
        SDL_Rect destination;
        calculateRects(texture, eye, source, destination);
        if (SDL_RenderCopy(renderer, texture.texture, &source, &destination) != 0)
        {
            RCLCPP_ERROR(get_logger(), "SDL_RenderCopy failed: %s", SDL_GetError());
        }
    }

    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &left,
                       const sensor_msgs::msg::Image::ConstSharedPtr &right)
    {
        const auto callback_start = std::chrono::steady_clock::now();
        const auto callback_ros_time = now();

        if (config_.mono)
        {
            if (!uploadImage(main_view_.renderer, left_texture_, *left, left_scratch_))
            {
                return;
            }
            const auto upload_end = std::chrono::steady_clock::now();
            SDL_RenderClear(main_view_.renderer);
            copyTexture(main_view_.renderer, left_texture_,
                        SDL_Rect{0, 0, config_.width, config_.height});
            SDL_RenderPresent(main_view_.renderer);
            const auto render_end = std::chrono::steady_clock::now();
            if (profile_)
            {
                recordProfile(*left, *right, callback_ros_time, callback_start,
                              upload_end, render_end);
            }
            return;
        }

        if (config_.concat)
        {
            if (!uploadImage(main_view_.renderer, left_texture_, *left, left_scratch_) ||
                !uploadImage(main_view_.renderer, right_texture_, *right, right_scratch_))
            {
                return;
            }
            const auto upload_end = std::chrono::steady_clock::now();
            SDL_RenderClear(main_view_.renderer);
            copyTexture(main_view_.renderer, left_texture_,
                        SDL_Rect{0, 0, config_.width, config_.height});
            copyTexture(main_view_.renderer, right_texture_,
                        SDL_Rect{config_.width, 0, config_.width, config_.height});
            SDL_RenderPresent(main_view_.renderer);
            const auto render_end = std::chrono::steady_clock::now();
            if (profile_)
            {
                recordProfile(*left, *right, callback_ros_time, callback_start,
                              upload_end, render_end);
            }
            return;
        }

        if (!uploadImage(main_view_.renderer, left_texture_, *left, left_scratch_) ||
            !uploadImage(right_view_.renderer, right_texture_, *right, right_scratch_))
        {
            return;
        }
        const auto upload_end = std::chrono::steady_clock::now();
        SDL_RenderClear(main_view_.renderer);
        copyTexture(main_view_.renderer, left_texture_,
                    SDL_Rect{0, 0, config_.width, config_.height});
        SDL_RenderPresent(main_view_.renderer);

        SDL_RenderClear(right_view_.renderer);
        copyTexture(right_view_.renderer, right_texture_,
                    SDL_Rect{0, 0, config_.width, config_.height});
        SDL_RenderPresent(right_view_.renderer);
        const auto render_end = std::chrono::steady_clock::now();
        if (profile_)
        {
            recordProfile(*left, *right, callback_ros_time, callback_start,
                          upload_end, render_end);
        }
    }

    void recordProfile(
        const sensor_msgs::msg::Image &left,
        const sensor_msgs::msg::Image &right,
        const rclcpp::Time &callback_ros_time,
        const std::chrono::steady_clock::time_point &callback_start,
        const std::chrono::steady_clock::time_point &upload_end,
        const std::chrono::steady_clock::time_point &render_end)
    {
        const auto milliseconds = [](const auto &duration) {
            return std::chrono::duration<double, std::milli>(duration).count();
        };
        const rclcpp::Time left_stamp(left.header.stamp);
        const rclcpp::Time right_stamp(right.header.stamp);
        const double input_age_ms = 1000.0 * std::max(
            (callback_ros_time - left_stamp).seconds(),
            (callback_ros_time - right_stamp).seconds());
        const double pair_skew_ms =
            1000.0 * std::abs((left_stamp - right_stamp).seconds());
        const double upload_ms = milliseconds(upload_end - callback_start);
        const double render_ms = milliseconds(render_end - upload_end);
        const double callback_ms = milliseconds(render_end - callback_start);

        if (profile_frames_ == 0)
        {
            profile_window_start_ = callback_start;
        }
        ++profile_frames_;
        profile_input_age_ms_ += input_age_ms;
        profile_pair_skew_ms_ += pair_skew_ms;
        profile_upload_ms_ += upload_ms;
        profile_render_ms_ += render_ms;
        profile_callback_ms_ += callback_ms;
        profile_max_present_age_ms_ = std::max(
            profile_max_present_age_ms_, input_age_ms + callback_ms);

        const double window_seconds =
            std::chrono::duration<double>(render_end - profile_window_start_).count();
        if (window_seconds >= 1.0)
        {
            const double count = static_cast<double>(profile_frames_);
            RCLCPP_INFO(
                get_logger(),
                "profile: %.1f fps, input age %.2f ms, pair skew %.2f ms, "
                "upload %.2f ms, render/present %.2f ms, callback %.2f ms, "
                "max presented age %.2f ms",
                count / window_seconds,
                profile_input_age_ms_ / count,
                profile_pair_skew_ms_ / count,
                profile_upload_ms_ / count,
                profile_render_ms_ / count,
                profile_callback_ms_ / count,
                profile_max_present_age_ms_);
            profile_frames_ = 0;
            profile_input_age_ms_ = 0.0;
            profile_pair_skew_ms_ = 0.0;
            profile_upload_ms_ = 0.0;
            profile_render_ms_ = 0.0;
            profile_callback_ms_ = 0.0;
            profile_max_present_age_ms_ = 0.0;
        }
    }


    Config config_;
    View main_view_;
    View right_view_;
    TextureState left_texture_;
    TextureState right_texture_;
    std::vector<std::uint8_t> left_scratch_;
    std::vector<std::uint8_t> right_scratch_;
    std::set<std::string> reported_encodings_;
    bool profile_ = false;
    std::chrono::steady_clock::time_point profile_window_start_;
    std::uint64_t profile_frames_ = 0;
    double profile_input_age_ms_ = 0.0;
    double profile_pair_skew_ms_ = 0.0;
    double profile_upload_ms_ = 0.0;
    double profile_render_ms_ = 0.0;
    double profile_callback_ms_ = 0.0;
    double profile_max_present_age_ms_ = 0.0;
    image_transport::SubscriberFilter left_sub_;
    image_transport::SubscriberFilter right_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
};

}  // namespace

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    if (SDL_Init(SDL_INIT_VIDEO) != 0)
    {
        std::cerr << "SDL could not initialize: " << SDL_GetError() << '\n';
        rclcpp::shutdown();
        return 1;
    }

    int result = 0;
    try
    {
        const Config config = processArgs(argc, argv);
        auto node = std::make_shared<StereoViewerNode>(config);
        rclcpp::WallRate event_rate(240.0);
        while (rclcpp::ok() && node->processEvents())
        {
            rclcpp::spin_some(node);
            event_rate.sleep();
        }
        node.reset();  // Destroy all SDL objects before SDL_Quit().
    }
    catch (const std::exception &error)
    {
        std::cerr << "display_video: " << error.what() << '\n';
        result = 1;
    }

    SDL_Quit();
    if (rclcpp::ok())
    {
        rclcpp::shutdown();
    }
    return result;
}
