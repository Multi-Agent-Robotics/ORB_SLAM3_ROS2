#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <cv_bridge/cv_bridge.h>


// ORB_SLAM3
#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "Eigen/Core" // Eigen::Vector3f
#include "sophus/se3.hpp" // Sophus::SE3<float>


#include "utility.hpp"

using std::string;

using ImuMsg = sensor_msgs::msg::Imu;
using ImageMsg = sensor_msgs::msg::Image;
using PoseMsg = geometry_msgs::msg::Pose;
using PoseStampedMsg = geometry_msgs::msg::PoseStamped;

template <typename T>
using Subscriber = rclcpp::Subscription<T>;

template <typename T>
using Publisher = rclcpp::Publisher<T>;


class StereoInertialNode final : public rclcpp::Node
{ 
public:
    StereoInertialNode(ORB_SLAM3::System* pSLAM, const string &settings_filepath, bool do_rectify, bool do_equalize);
    ~StereoInertialNode();

private:
    auto grab_imu(const ImuMsg::SharedPtr msg) -> void;
    // void GrabImu(const ImuMsg::SharedPtr msg);
    auto grab_image_left(const ImageMsg::SharedPtr msg) -> void;
    // void GrabImageLeft(const ImageMsg::SharedPtr msgLeft);
    auto grab_image_right(const ImageMsg::SharedPtr msg) -> void;
    // void GrabImageRight(const ImageMsg::SharedPtr msgRight);
    auto get_image(const ImageMsg::SharedPtr msg) -> cv::Mat;
    // cv::Mat GetImage(const ImageMsg::SharedPtr msg);
    auto sync_with_imu() -> void;
    // void SyncWithImu();

    // auto pub_imu_callback() -> void;

    // auto pub_tracked_map_points() -> void;
    Subscriber<ImuMsg>::SharedPtr sub_imu;
    Subscriber<ImageMsg>::SharedPtr sub_img_left;
    Subscriber<ImageMsg>::SharedPtr sub_img_right;

    // rclcpp::TimerBase::SharedPtr pub_imu_timer;
    // Publisher<ImuMsg>::SharedPtr pub_imu;

    // Publisher<> pub_tracked_map_points;
    // rclcpp::TimerBase::SharedPtr pub_tracked_map_points_timer;

    auto pub_camera_pose_callback() -> void;
    Publisher<PoseStampedMsg>::SharedPtr pub_camera_pose;
    rclcpp::TimerBase::SharedPtr pub_camera_pose_timer;

    // ORB_SLAM3::System *SLAM_;
    ORB_SLAM3::System *orbslam3_system;
    std::thread *sync_thread;

    // IMU
    queue<ImuMsg::SharedPtr> imubuf;
    std::mutex mutex_imu;

    // Image
    queue<ImageMsg::SharedPtr> img_left_buf, img_right_buf;
    std::mutex mutex_img_left, mutex_img_right;

    bool do_rectify;
    bool do_equalize;
    cv::Mat M1l_, M2l_, M1r_, M2r_;

    // bool bClahe_;
    bool apply_clahe;
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
};
