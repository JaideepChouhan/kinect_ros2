// Copyright 2024 ROS2 Community -- Apache-2.0
//
// kinect_tilt_accel_node.cpp
//
// Kinect Xbox 360 Model 1473 – Tilt Motor & Accelerometer Driver
//
// WHY C++ IS REQUIRED FOR THE 1473
// ──────────────────────────────────
// The 1473 sets motor_control_with_audio_enabled = true (from tilt.c).
// All tilt and accelerometer operations go through the AUDIO USB interface
// (usb_audio.dev, endpoint 0x01/0x81) via libusb bulk transfers:
//
//   freenect_update_tilt_state(dev)  → calls update_tilt_state_alt(dev)
//                                        → libusb_bulk_transfer(usb_audio.dev, …)
//   freenect_set_tilt_degs(dev,ang)  → calls freenect_set_tilt_degs_alt(dev,ang)
//                                        → libusb_bulk_transfer(usb_audio.dev, …)
//
// The Python freenect bindings do NOT expose this code path correctly.
// freenect.sync_get_accel() does not exist; the state-based API never
// triggers the alt path for 1473.  C++ with the full libfreenect C API
// is the only reliable approach.
//
// IMPORTANT – USB CONFLICT WITH kinect_audio_node
// ─────────────────────────────────────────────────
// This node opens the device with FREENECT_DEVICE_AUDIO (needed for 1473).
// The kinect_audio_node ALSO opens with FREENECT_DEVICE_AUDIO.
// They CANNOT run simultaneously → LIBUSB_ERROR_BUSY.
//
// Solutions:
//   A) Run this node standalone (no audio streaming).
//   B) Use kinect_audio_motor_node.cpp (the combined node) instead – it
//      handles audio streaming AND tilt/accel from one device handle.
//
// Published topics
// ─────────────────
//   /kinect/tilt_angle  (std_msgs/Float32)         – current tilt in degrees
//   /kinect/accel       (geometry_msgs/Vector3)    – acceleration in m/s²
//   /kinect/imu/data    (sensor_msgs/Imu)          – full IMU message
//
// Subscribed topics
// ──────────────────
//   /kinect/tilt_cmd    (std_msgs/Float32)         – desired tilt in degrees
//
// Physics from tilt.c
// ────────────────────
//   FREENECT_COUNTS_PER_G = 819  (raw int16 counts per 1g)
//   GRAVITY               = 9.80665 m/s²
//   mks_accel = raw / 819 × 9.80665  [m/s²]
//
// Parameters
// ───────────
//   device_index     int    0      Kinect device index
//   publish_rate_hz  float  10.0   Accel/tilt publish rate (Hz)
//   tilt_min_deg     float -27.0   Safety clamp min
//   tilt_max_deg     float  27.0   Safety clamp max

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/imu.hpp>

extern "C" {
#include <libfreenect/libfreenect.h>
}

#include <chrono>
#include <mutex>
#include <stdexcept>
#include <string>

using namespace std::chrono_literals;
using Float32  = std_msgs::msg::Float32;
using Vector3  = geometry_msgs::msg::Vector3;
using Imu      = sensor_msgs::msg::Imu;


class KinectTiltAccelNode : public rclcpp::Node
{
public:
    KinectTiltAccelNode()
    : Node("kinect_tilt_accel_node"),
      ctx_(nullptr),
      dev_(nullptr)
    {
        // ── Parameters ──────────────────────────────────────────────────
        device_index_  = declare_parameter<int>("device_index",     0);
        double rate    = declare_parameter<double>("publish_rate_hz", 10.0);
        tilt_min_      = declare_parameter<double>("tilt_min_deg",   -27.0);
        tilt_max_      = declare_parameter<double>("tilt_max_deg",    27.0);

        // ── Publishers ───────────────────────────────────────────────────
        pub_tilt_  = create_publisher<Float32>("/kinect/tilt_angle", 10);
        pub_accel_ = create_publisher<Vector3>("/kinect/accel",      10);
        pub_imu_   = create_publisher<Imu>    ("/kinect/imu/data",   10);

        // ── Subscriber ───────────────────────────────────────────────────
        sub_cmd_ = create_subscription<Float32>(
            "/kinect/tilt_cmd", 10,
            [this](const Float32::SharedPtr msg) {
                this->on_tilt_cmd(msg->data);
            }
        );

        // ── libfreenect init ─────────────────────────────────────────────
        if (freenect_init(&ctx_, nullptr) < 0) {
            RCLCPP_FATAL(get_logger(), "freenect_init() failed");
            throw std::runtime_error("freenect_init failed");
        }

        freenect_set_log_level(ctx_, FREENECT_LOG_ERROR);

        // FREENECT_DEVICE_AUDIO is required for the 1473:
        //   Opening the audio subdevice uploads the audio firmware and sets
        //   dev->motor_control_with_audio_enabled = true inside libfreenect.
        //   After that, freenect_update_tilt_state() and freenect_set_tilt_degs()
        //   automatically use the alt path (usb_audio.dev bulk transfers).
        //
        // On the 1414, FREENECT_DEVICE_MOTOR alone is sufficient, but opening
        // with AUDIO also works because motor_control_with_audio_enabled stays
        // false on 1414 devices and the standard path is used as fallback.
        freenect_select_subdevices(
            ctx_,
            static_cast<freenect_device_flags>(FREENECT_DEVICE_MOTOR |
                                               FREENECT_DEVICE_AUDIO)
        );

        int n = freenect_num_devices(ctx_);
        RCLCPP_INFO(get_logger(), "libfreenect detected %d Kinect device(s)", n);

        if (n == 0) {
            freenect_shutdown(ctx_);
            RCLCPP_FATAL(get_logger(),
                "No Kinect device found.\n"
                "  • Make sure kinect_driver_node ran first (firmware upload)\n"
                "  • Check: lsusb | grep 045e\n"
                "  • Check udev rules for 045e:02ad");
            throw std::runtime_error("No Kinect device found");
        }

        if (freenect_open_device(ctx_, &dev_, device_index_) < 0) {
            freenect_shutdown(ctx_);
            RCLCPP_FATAL(get_logger(),
                "Failed to open Kinect device %d.\n"
                "  • Check: ls -la /dev/bus/usb/*/*\n"
                "  • udev rule: ATTR{idProduct}==\"02ad\", GROUP=\"audio\"\n"
                "  • sudo usermod -aG audio $USER  (then re-login)",
                device_index_);
            throw std::runtime_error("freenect_open_device failed");
        }

        // One initial tilt-state read to verify the connection works.
        // This also triggers the firmware handshake on 1473.
        {
            std::lock_guard<std::mutex> lk(mx_);
            int rc = freenect_update_tilt_state(dev_);
            if (rc < 0) {
                RCLCPP_WARN(get_logger(),
                    "Initial freenect_update_tilt_state returned %d. "
                    "Motor may not be fully initialised yet.", rc);
            } else {
                freenect_raw_tilt_state* st = freenect_get_tilt_state(dev_);
                if (st) {
                    RCLCPP_INFO(get_logger(),
                        "Initial tilt angle: %.1f°", freenect_get_tilt_degs(st));
                }
            }
        }

        // ── Periodic publish timer ────────────────────────────────────────
        auto period = std::chrono::duration<double>(1.0 / rate);
        timer_ = create_timer(period, [this]() { this->read_and_publish(); });

        RCLCPP_INFO(get_logger(),
            "KinectTiltAccelNode ready (model 1473 compatible)\n"
            "  /kinect/tilt_angle  – degrees [%.0f, %.0f]\n"
            "  /kinect/accel       – m/s²  (FREENECT_COUNTS_PER_G = 819)\n"
            "  /kinect/imu/data    – sensor_msgs/Imu (accel only, no gyro)\n"
            "  /kinect/tilt_cmd    – subscribe to set tilt",
            tilt_min_, tilt_max_);
    }

    ~KinectTiltAccelNode() override
    {
        if (dev_) {
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
    //  Periodic read
    // ════════════════════════════════════════════════════════════════════

    void read_and_publish()
    {
        std::lock_guard<std::mutex> lk(mx_);

        // freenect_update_tilt_state():
        //   1473 → update_tilt_state_alt() → libusb_bulk_transfer on usb_audio.dev
        //   1414 → fnusb_control() on usb_motor
        int rc = freenect_update_tilt_state(dev_);
        if (rc < 0) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "freenect_update_tilt_state failed (%d). "
                "Is the Kinect powered and connected?", rc);
            return;
        }

        freenect_raw_tilt_state* state = freenect_get_tilt_state(dev_);
        if (!state) {
            RCLCPP_WARN(get_logger(), "freenect_get_tilt_state returned null");
            return;
        }

        // ── Tilt angle ─────────────────────────────────────────────────
        double angle_deg = freenect_get_tilt_degs(state);

        Float32 tilt_msg;
        tilt_msg.data = static_cast<float>(angle_deg);
        pub_tilt_->publish(tilt_msg);

        // ── Accelerometer in m/s²  (from tilt.c: raw / 819 × 9.80665) ─
        double ax, ay, az;
        freenect_get_mks_accel(state, &ax, &ay, &az);

        Vector3 accel_msg;
        accel_msg.x = ax;
        accel_msg.y = ay;
        accel_msg.z = az;
        pub_accel_->publish(accel_msg);

        // ── IMU message ────────────────────────────────────────────────
        Imu imu_msg;
        rclcpp::Time now = get_clock()->now();
        imu_msg.header.stamp.sec = now.seconds();
        imu_msg.header.stamp.nanosec = now.nanoseconds();
        imu_msg.header.frame_id = "kinect_imu_frame";

        // Linear acceleration (m/s²)
        imu_msg.linear_acceleration.x = ax;
        imu_msg.linear_acceleration.y = ay;
        imu_msg.linear_acceleration.z = az;

        // Covariance for acceleration (estimated from Kinect datasheet)
        imu_msg.linear_acceleration_covariance = {
            1e-4, 0.0,  0.0,
            0.0,  1e-4, 0.0,
            0.0,  0.0,  1e-4
        };

        // No gyroscope on the Kinect – signal "not available" per REP 145
        imu_msg.angular_velocity_covariance[0] = -1.0;
        imu_msg.orientation_covariance[0]      = -1.0;

        pub_imu_->publish(imu_msg);

        RCLCPP_DEBUG(get_logger(),
            "tilt=%.1f°  accel=[%.3f, %.3f, %.3f] m/s²",
            angle_deg, ax, ay, az);
    }

    // ════════════════════════════════════════════════════════════════════
    //  Tilt command subscriber
    // ════════════════════════════════════════════════════════════════════

    void on_tilt_cmd(float angle_f)
    {
        double angle = std::max(tilt_min_, std::min(tilt_max_,
                                static_cast<double>(angle_f)));

        std::lock_guard<std::mutex> lk(mx_);

        // freenect_set_tilt_degs():
        //   1473 → freenect_set_tilt_degs_alt() → libusb_bulk_transfer on usb_audio.dev
        //   1414 → fnusb_control() on usb_motor
        int rc = freenect_set_tilt_degs(dev_, angle);
        if (rc < 0) {
            RCLCPP_WARN(get_logger(),
                "freenect_set_tilt_degs(%.1f) returned %d", angle, rc);
        } else {
            RCLCPP_INFO(get_logger(), "Tilt → %.1f°", angle);
        }
    }

    // ════════════════════════════════════════════════════════════════════
    //  Members
    // ════════════════════════════════════════════════════════════════════

    freenect_context* ctx_;
    freenect_device*  dev_;
    std::mutex        mx_;          // protects all freenect calls
    int               device_index_;
    double            tilt_min_;
    double            tilt_max_;

    rclcpp::Publisher<Float32>::SharedPtr  pub_tilt_;
    rclcpp::Publisher<Vector3>::SharedPtr  pub_accel_;
    rclcpp::Publisher<Imu>::SharedPtr      pub_imu_;
    rclcpp::Subscription<Float32>::SharedPtr sub_cmd_;
    rclcpp::TimerBase::SharedPtr           timer_;
};


// ── Entry point ─────────────────────────────────────────────────────────────

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<KinectTiltAccelNode>());
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("kinect_tilt_accel"),
            "Fatal: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
