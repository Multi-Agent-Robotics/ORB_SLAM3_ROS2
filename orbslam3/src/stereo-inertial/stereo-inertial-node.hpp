#pragma once

// ROS2
#include "rclcpp/rclcpp.hpp"
#include <cv_bridge/cv_bridge.h>

// ROS2 msgs
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "orbslam3_msgs/msg/orb_features.hpp"
#include "orbslam3_msgs/msg/key_point.hpp"
#include "orbslam3_msgs/msg/descriptors.hpp"

// ORB_SLAM3
#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "Eigen/Core"     // Eigen::Vector3f
#include "sophus/se3.hpp" // Sophus::SE3<float>

// #include "utility.hpp"

namespace ros2 = rclcpp;
namespace orbslam3 = ORB_SLAM3;

using ImuMsg = sensor_msgs::msg::Imu;
using ImageMsg = sensor_msgs::msg::Image;
using PoseMsg = geometry_msgs::msg::Pose;
using PoseStampedMsg = geometry_msgs::msg::PoseStamped;
using OrbFeaturesMsg = orbslam3_msgs::msg::OrbFeatures;
using KeyPointMsg = orbslam3_msgs::msg::KeyPoint;

template <typename T>
using Subscriber = ros2::Subscription<T>;

template <typename T>
using Publisher = ros2::Publisher<T>;

class StereoInertialNode final : public ros2::Node
{

public:
    StereoInertialNode(const std::string &node_name, ORB_SLAM3::System *pSLAM, const std::string &settings_filepath, bool do_rectify, bool do_equalize);
    ~StereoInertialNode();

private:
    auto grab_imu(const ImuMsg::SharedPtr msg) -> void;
    auto grab_image_left(const ImageMsg::SharedPtr msg) -> void;
    auto grab_image_right(const ImageMsg::SharedPtr msg) -> void;
    auto get_image(const ImageMsg::SharedPtr msg) -> cv::Mat;
    auto sync_with_imu() -> void;

    // Camera images
    Subscriber<ImuMsg>::SharedPtr sub_imu;
    Subscriber<ImageMsg>::SharedPtr sub_img_left;
    Subscriber<ImageMsg>::SharedPtr sub_img_right;

    // ORB features
    Publisher<OrbFeaturesMsg>::SharedPtr pub_orb_features;
    ros2::TimerBase::SharedPtr pub_orb_features_timer;
    auto pub_orb_features_callback() -> void;

    // Publisher<> pub_tracked_map_points;
    // ros2::TimerBase::SharedPtr pub_tracked_map_points_timer;

    // auto pub_orb_features_from_current_frame_callback() -> void;
    // Publisher<KeypointsStampedMsg>::SharedPtr pub_orb_features_from_current_frame;
    // ros2::TimerBase::SharedPtr pub_orb_features_from_current_frame_timer;

    // Camera pose
    Publisher<PoseStampedMsg>::SharedPtr pub_camera_pose;
    ros2::TimerBase::SharedPtr pub_camera_pose_timer;
    auto pub_camera_pose_callback() -> void;

    ORB_SLAM3::System *orbslam3_system;
    std::thread *sync_thread;

    // IMU
    std::queue<ImuMsg::SharedPtr> imubuf;
    std::mutex mutex_imu;

    // Image
    std::queue<ImageMsg::SharedPtr> img_left_buf, img_right_buf;
    std::mutex mutex_img_left, mutex_img_right;

    bool do_rectify;
    bool do_equalize;
    cv::Mat M1l_, M2l_, M1r_, M2r_;

    bool apply_clahe;
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
};
