// Copyright 2024 ROS2 Community -- Apache-2.0
//
// kinect_audio_node.cpp
//
// Uses the libfreenect C audio API (libfreenect_audio.h) to access the
// Kinect v1 microphone array directly over USB, bypassing ALSA entirely.
//
// What we learned from libfreenect/src/audio.c:
// ──────────────────────────────────────────────
//  • freenect_start_audio() allocates fixed 256-sample buffers per channel:
//      dev->audio.mic_buffer[4]    = int32_t[256] each
//      dev->audio.cancelled_buffer = int16_t[256]
//
//  • The callback fires once per USB "window" change, always delivering
//    exactly 256 samples – at 16 kHz this means ~62.5 callbacks/second.
//
//  • Four separate int32 mic arrays  → /kinect/audio/raw (Int32MultiArray)
//    One int16 noise-cancelled array → /kinect/audio/cancelled (Int16MultiArray)
//    The cancelled channel is hardware beam-formed – best for STT.
//
//  • We open with FREENECT_DEVICE_AUDIO only.  From tilt.c we know that on
//    the Xbox 360 Kinect (1414 variant) motor_control_with_audio_enabled=false,
//    so motor uses its own USB path – no conflict with tilt_control_node.
//
// Topics published
// ─────────────────
//  /kinect/audio/raw       std_msgs/Int32MultiArray
//      layout.dim[0] label="frames"   size=256  stride=1024
//      layout.dim[1] label="channels" size=4    stride=4
//      data: [m0[0],m1[0],m2[0],m3[0], m0[1],m1[1],m2[1],m3[1], ...]
//
//  /kinect/audio/cancelled std_msgs/Int16MultiArray
//      layout.dim[0] label="samples" size=256 stride=256
//      data: [s0, s1, …, s255]   (hardware noise-cancelled mono)
//
//  /kinect/audio/info      std_msgs/String   (JSON, latched-like)
//
// Parameters
// ──────────
//  device_index  int  0   Kinect device index (0 = first connected device)

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

extern "C" {
#include <libfreenect/libfreenect.h>
#include <libfreenect/libfreenect_audio.h>
}

#include <atomic>
#include <chrono>
#include <stdexcept>
#include <string>
#include <thread>

using namespace std::chrono_literals;

// Constants confirmed from audio.c source
static constexpr int SAMPLES_PER_CB   = 256;   // always 256, hardcoded in freenect_start_audio
static constexpr int SAMPLE_RATE_HZ   = 16000;  // Kinect v1 fixed sample rate
static constexpr int NUM_MIC_CHANNELS = 4;       // mic_buffer[0..3]


class KinectAudioNode : public rclcpp::Node
{
public:
    KinectAudioNode()
    : Node("kinect_audio_node"),
      ctx_(nullptr),
      dev_(nullptr),
      running_(false),
      cb_count_(0)
    {
        // ── Parameters ──────────────────────────────────────────────────
        device_index_ = declare_parameter<int>("device_index", 0);

        // ── Publishers ───────────────────────────────────────────────────
        pub_raw_ = create_publisher<std_msgs::msg::Int32MultiArray>(
            "/kinect/audio/raw", 10);
        pub_cancelled_ = create_publisher<std_msgs::msg::Int16MultiArray>(
            "/kinect/audio/cancelled", 10);
        pub_info_ = create_publisher<std_msgs::msg::String>(
            "/kinect/audio/info", 10);

        // ── libfreenect init ─────────────────────────────────────────────
        if (freenect_init(&ctx_, nullptr) < 0) {
            RCLCPP_FATAL(get_logger(), "freenect_init() failed");
            throw std::runtime_error("freenect_init failed");
        }

        // Suppress verbose libfreenect output – only show real errors
        freenect_set_log_level(ctx_, FREENECT_LOG_ERROR);

        // Select ONLY the audio subdevice (USB 045e:02ad audio interface).
        // This avoids any conflict with kinect_driver_node (FREENECT_DEVICE_CAMERA)
        // and tilt_control_node (which on 1414 uses its own FREENECT_DEVICE_MOTOR path).
        freenect_select_subdevices(ctx_, FREENECT_DEVICE_AUDIO);

        int num_devices = freenect_num_devices(ctx_);
        RCLCPP_INFO(get_logger(), "libfreenect found %d Kinect device(s)", num_devices);

        if (num_devices == 0) {
            freenect_shutdown(ctx_);
            RCLCPP_FATAL(get_logger(),
                "No Kinect device found.\n"
                "  Check: lsusb | grep 045e\n"
                "  If kinect_driver_node is not running, start it first so\n"
                "  the audio firmware gets uploaded to the device.");
            throw std::runtime_error("No Kinect devices found");
        }

        if (freenect_open_device(ctx_, &dev_, device_index_) < 0) {
            freenect_shutdown(ctx_);
            RCLCPP_FATAL(get_logger(),
                "Failed to open Kinect device %d for audio.\n"
                "  Fix udev rules:\n"
                "    echo 'SUBSYSTEM==\"usb\", ATTR{idVendor}==\"045e\",\n"
                "      ATTR{idProduct}==\"02ad\", MODE=\"0664\", GROUP=\"audio\"'\n"
                "      | sudo tee /etc/udev/rules.d/52-kinect-audio.rules\n"
                "    sudo udevadm control --reload-rules && sudo udevadm trigger\n"
                "    sudo usermod -aG audio $USER   # then log out/in",
                device_index_);
            throw std::runtime_error("freenect_open_device failed");
        }

        // Store `this` so the static C callback can reach the node instance.
        freenect_set_user(dev_, this);

        // Register the audio-in callback defined in audio.c.
        freenect_set_audio_in_callback(dev_, &KinectAudioNode::audio_in_cb_static);

        // freenect_start_audio():
        //   - allocates mic_buffer[4] (int32, 256 each) and cancelled_buffer (int16, 256)
        //   - starts the isochronous USB IN stream on endpoint 0x82
        //   - starts the isochronous USB OUT stream on endpoint 0x02
        if (freenect_start_audio(dev_) < 0) {
            freenect_close_device(dev_);
            freenect_shutdown(ctx_);
            RCLCPP_FATAL(get_logger(), "freenect_start_audio() failed");
            throw std::runtime_error("freenect_start_audio failed");
        }

        // Publish metadata (repeated several times so late subscribers get it)
        publish_info_latch();

        // Start the freenect event loop in a dedicated thread.
        // rclcpp::Publisher::publish() is thread-safe, so calling it from
        // within the freenect thread is safe.
        running_ = true;
        freenect_thread_ = std::thread(&KinectAudioNode::freenect_loop, this);

        RCLCPP_INFO(get_logger(),
            "KinectAudioNode ready  ──  libfreenect audio API\n"
            "  /kinect/audio/raw       4-ch int32, %d samples/msg @ %.1f Hz\n"
            "  /kinect/audio/cancelled 1-ch int16, %d samples/msg (HW noise-cancelled)\n"
            "  /kinect/audio/info      JSON metadata",
            SAMPLES_PER_CB,
            static_cast<double>(SAMPLE_RATE_HZ) / SAMPLES_PER_CB,
            SAMPLES_PER_CB);
    }

    ~KinectAudioNode() override
    {
        running_ = false;
        if (freenect_thread_.joinable()) {
            freenect_thread_.join();
        }
        if (dev_) {
            freenect_stop_audio(dev_);   // stops ISO streams, frees mic/cancelled buffers
            freenect_close_device(dev_);
            dev_ = nullptr;
        }
        if (ctx_) {
            freenect_shutdown(ctx_);
            ctx_ = nullptr;
        }
    }

private:
    // ════════════════════════════════════════════════════════════════════
    //  Audio callback
    // ════════════════════════════════════════════════════════════════════

    // Static trampoline required by the libfreenect C callback API.
    // Called from the freenect event-loop thread once per USB window change.
    // From audio.c: num_samples is always 256.
    static void audio_in_cb_static(freenect_device* dev,
                                   int              num_samples,
                                   int32_t*         mic1,
                                   int32_t*         mic2,
                                   int32_t*         mic3,
                                   int32_t*         mic4,
                                   int16_t*         cancelled,
                                   void*            /*unknown*/)
    {
        auto* self = static_cast<KinectAudioNode*>(freenect_get_user(dev));
        if (self) {
            self->audio_in_cb(num_samples, mic1, mic2, mic3, mic4, cancelled);
        }
    }

    // Member version: builds and publishes the two ROS messages.
    void audio_in_cb(int      num_samples,
                     int32_t* mic1,
                     int32_t* mic2,
                     int32_t* mic3,
                     int32_t* mic4,
                     int16_t* cancelled)
    {
        // ── /kinect/audio/raw  (4 × int32, interleaved) ─────────────────
        {
            auto msg = std::make_unique<std_msgs::msg::Int32MultiArray>();

            // dim[0] = frames (time axis), dim[1] = channels
            msg->layout.data_offset = 0;
            msg->layout.dim.resize(2);
            msg->layout.dim[0].label  = "frames";
            msg->layout.dim[0].size   = static_cast<uint32_t>(num_samples);
            msg->layout.dim[0].stride = static_cast<uint32_t>(num_samples * NUM_MIC_CHANNELS);
            msg->layout.dim[1].label  = "channels";
            msg->layout.dim[1].size   = NUM_MIC_CHANNELS;
            msg->layout.dim[1].stride = NUM_MIC_CHANNELS;

            msg->data.resize(static_cast<std::size_t>(num_samples * NUM_MIC_CHANNELS));
            for (int i = 0; i < num_samples; ++i) {
                msg->data[static_cast<std::size_t>(i * 4 + 0)] = mic1[i];
                msg->data[static_cast<std::size_t>(i * 4 + 1)] = mic2[i];
                msg->data[static_cast<std::size_t>(i * 4 + 2)] = mic3[i];
                msg->data[static_cast<std::size_t>(i * 4 + 3)] = mic4[i];
            }

            pub_raw_->publish(std::move(msg));
        }

        // ── /kinect/audio/cancelled  (1 × int16, hardware noise-cancelled) ──
        {
            auto msg = std::make_unique<std_msgs::msg::Int16MultiArray>();

            msg->layout.data_offset = 0;
            msg->layout.dim.resize(1);
            msg->layout.dim[0].label  = "samples";
            msg->layout.dim[0].size   = static_cast<uint32_t>(num_samples);
            msg->layout.dim[0].stride = static_cast<uint32_t>(num_samples);

            // Direct copy from libfreenect's cancelled_buffer
            msg->data.assign(cancelled, cancelled + num_samples);

            pub_cancelled_->publish(std::move(msg));
        }

        // Periodic diagnostics
        uint64_t count = ++cb_count_;
        if (count % 625 == 0) {   // every ~10 s at 62.5 Hz
            double recorded_s =
                static_cast<double>(count * SAMPLES_PER_CB) / SAMPLE_RATE_HZ;
            RCLCPP_INFO(get_logger(),
                "Audio  ▸  %lu callbacks  %.0f s captured",
                count, recorded_s);
        }
    }

    // ════════════════════════════════════════════════════════════════════
    //  freenect event loop
    // ════════════════════════════════════════════════════════════════════

    void freenect_loop()
    {
        RCLCPP_INFO(get_logger(), "freenect audio event loop started");
        struct timeval timeout { 0, 100'000 };  // 100 ms – lets us check running_
        while (running_) {
            int rc = freenect_process_events_timeout(ctx_, &timeout);
            if (rc < 0 && running_) {
                RCLCPP_ERROR(get_logger(),
                    "freenect_process_events_timeout error: %d", rc);
                break;
            }
        }
        RCLCPP_INFO(get_logger(), "freenect audio event loop exited");
    }

    // ════════════════════════════════════════════════════════════════════
    //  Metadata
    // ════════════════════════════════════════════════════════════════════

    void publish_info_latch()
    {
        std_msgs::msg::String msg;
        msg.data =
            "{"
            "\"sample_rate\":"      + std::to_string(SAMPLE_RATE_HZ)           + ","
            "\"samples_per_cb\":"   + std::to_string(SAMPLES_PER_CB)           + ","
            "\"cb_rate_hz\":"       + std::to_string(SAMPLE_RATE_HZ / SAMPLES_PER_CB) + ","
            "\"mic_channels\":4,"
            "\"raw_dtype\":\"int32\","
            "\"cancelled_dtype\":\"int16\","
            "\"raw_topic\":\"/kinect/audio/raw\","
            "\"cancelled_topic\":\"/kinect/audio/cancelled\""
            "}";

        // Publish several times (poor-man's latching) so late subscribers receive it
        for (int i = 0; i < 10; ++i) {
            pub_info_->publish(msg);
            std::this_thread::sleep_for(50ms);
        }
    }

    // ════════════════════════════════════════════════════════════════════
    //  Member variables
    // ════════════════════════════════════════════════════════════════════

    freenect_context*  ctx_;
    freenect_device*   dev_;
    std::thread        freenect_thread_;
    std::atomic<bool>  running_;
    std::atomic<uint64_t> cb_count_;
    int                device_index_;

    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_raw_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr pub_cancelled_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr          pub_info_;
};


// ── Entry point ─────────────────────────────────────────────────────────────

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<KinectAudioNode>());
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("kinect_audio"),
            "Node terminated with exception: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
