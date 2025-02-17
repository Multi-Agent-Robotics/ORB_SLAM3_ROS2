#include "stereo-inertial-node.hpp"

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <mutex>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "cv-utils.hpp"
#include "log-macro.hpp"
#include "utils.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

using u8 = std::uint8_t;
using u16 = std::uint16_t;
using u32 = std::uint32_t;
using u64 = std::uint64_t;
using i8 = std::int8_t;
using i16 = std::int16_t;
using i32 = std::int32_t;
using i64 = std::int64_t;
using f32 = float;
using f64 = double;

template <typename T> using Vec = std::vector<T>;
using String = std::string;

StereoInertialNode::StereoInertialNode(const std::string &node_name,
                                       ORB_SLAM3::System *SLAM,
                                       const std::string &settings_filepath,
                                       bool do_rectify_, bool do_equalize_)
    : Node(node_name), orbslam3_system(SLAM), do_rectify(do_rectify_),
      do_equalize(do_equalize_), apply_clahe(do_equalize_) {
    if (this->do_rectify) {
        DEBUG_LOG(stderr, "applying rectification");
        // Load settings related to stereo calibration
        auto settings =
            cv::FileStorage(settings_filepath, cv::FileStorage::READ);
        if (!settings.isOpened()) {
            std::fprintf(stderr, "ERROR: Wrong path to settings, %s\n",
                         settings_filepath.c_str());
            std::exit(1);
        }

        cv::Mat left_K, right_K, left_P, right_P, left_R, right_R, left_D,
            right_D;

        settings["LEFT.K"] >> left_K;
        settings["RIGHT.K"] >> right_K;

        settings["LEFT.P"] >> left_P;
        settings["RIGHT.P"] >> right_P;

        settings["LEFT.R"] >> left_R;
        settings["RIGHT.R"] >> right_R;

        settings["LEFT.D"] >> left_D;
        settings["RIGHT.D"] >> right_D;

        int rows_l = settings["LEFT.height"];
        int cols_l = settings["LEFT.width"];
        int rows_r = settings["RIGHT.height"];
        int cols_r = settings["RIGHT.width"];

        if (left_K.empty() || right_K.empty() || left_P.empty() ||
            right_P.empty() || left_R.empty() || right_R.empty() ||
            left_D.empty() || right_D.empty() || rows_l == 0 || rows_r == 0 ||
            cols_l == 0 || cols_r == 0) {
            std::fprintf(stderr, "ERROR: Calibration parameters to rectify "
                                 "stereo are missing!\n");
            std::exit(1);
        }

        cv::initUndistortRectifyMap(
            left_K, left_D, left_R, left_P.rowRange(0, 3).colRange(0, 3),
            cv::Size(cols_l, rows_l), CV_32F, M1l_, M2l_);
        cv::initUndistortRectifyMap(
            right_K, right_D, right_R, right_P.rowRange(0, 3).colRange(0, 3),
            cv::Size(cols_r, rows_r), CV_32F, M1r_, M2r_);
    }

    auto keeplast = [](std::size_t) -> rclcpp::QoS {
        return rclcpp::QoS(rclcpp::KeepLast(100));
    };

    this->sub_imu = this->create_subscription<ImuMsg>(
        "imu", keeplast(100),
        std::bind(&StereoInertialNode::grab_imu, this, _1));
    this->sub_img_left = this->create_subscription<ImageMsg>(
        "camera/left", keeplast(100),
        std::bind(&StereoInertialNode::grab_image_left, this, _1));
    this->sub_img_right = this->create_subscription<ImageMsg>(
        "camera/right", keeplast(100),
        std::bind(&StereoInertialNode::grab_image_right, this, _1));

    this->sync_thread =
        new std::thread(&StereoInertialNode::sync_with_imu, this);

    const auto topic_ns_prefix = std::string("orbslam3");

    // Publishers
    // --------------------------------------------------------------------------------
    // {
    //     std::string topic_name = topic_ns_prefix +
    //     "orb_features_from_current_frame"; auto qos =
    //     rclcpp::QoS(rclcpp::KeepLast(10));
    //     this->pub_orb_features_from_current_frame =
    //     this->create_publisher<ORBFeaturesMsg>(topic_name, qos);
    //     this->pub_orb_features_from_current_frame_timer =
    //     this->create_wall_timer(
    //         100ms,
    //         std::bind(&StereoInertialNode::pub_orb_features_from_current_frame_callback,
    //         this));
    // }
    // {
    //     const std::string topic_name =
    //     utils::format_topic_path(topic_ns_prefix, "camera", "pose"); const
    //     auto qos = rclcpp::QoS(rclcpp::KeepLast(10)); this->pub_camera_pose =
    //     this->create_publisher<PoseStampedMsg>(topic_name, qos);
    //     this->pub_camera_pose_timer = this->create_wall_timer(
    //         100ms,
    //         std::bind(&StereoInertialNode::pub_camera_pose_callback, this));
    // }

    // {
    //     const std::string topic_name =
    //     utils::format_topic_path(topic_ns_prefix, "orb_features"); const auto
    //     qos = rclcpp::QoS(rclcpp::KeepLast(10)); this->pub_orb_features =
    //     this->create_publisher<OrbFeaturesMsg>(topic_name, qos);
    //     this->pub_orb_features_timer = this->create_wall_timer(
    //         100ms,
    //         std::bind(&StereoInertialNode::pub_orb_features_callback, this));
    // }
    // this->pub_orb_features_timer = this->create_wall_timer(
    //     100ms,
    //     std::bind(&StereoInertialNode::pub_orb_features_callback, this));
}

// auto StereoInertialNode::pub_orb_features_callback() -> void {
//     static std::uint64_t times_called = 0;
//     times_called += 1;
//     ORB_SLAM3::Tracking *tracker = this->orbslam3_system->get_tracker();
//     ORB_SLAM3::Frame &current_frame = tracker->mCurrentFrame;
//     // ORB_SLAM3::Frame& last_frame = tracker->mLastFrame;
//     std::vector<cv::KeyPoint> &keypoints_left_img = current_frame.mvKeys;
//     std::vector<cv::KeyPoint> &keypoints_right = current_frame.mvKeysRight;

//     cv::Mat &descriptors_left_img = current_frame.mDescriptors;
//     cv::Mat &descriptors_right = current_frame.mDescriptorsRight;

//     auto msg = OrbFeaturesMsg();
//     for (auto &keypoint : keypoints_left_img) {
//         auto kp = KeyPointMsg();
//         kp.x = keypoint.pt.x;
//         kp.y = keypoint.pt.y;
//         kp.size = keypoint.size;
//         kp.angle = keypoint.angle;
//         kp.response = keypoint.response;
//         kp.octave = keypoint.octave;
//         kp.class_id = keypoint.class_id;
//         msg.keypoints.push_back(kp);
//     }

//     msg.descriptors.rows = descriptors_left_img.rows;
//     msg.descriptors.columns = descriptors_left_img.cols;
//     msg.descriptors.data.reserve(descriptors_left_img.rows *
//                                  descriptors_left_img.cols);
//     // A cv::Mat is stored in row-major order, so by taking an iterator over
//     the
//     // data, we are iterating over the rows of the matrix. This way the array
//     is
//     // flattened.
//     msg.descriptors.data.assign(descriptors_left_img.begin<u8>(),
//                                 descriptors_left_img.end<u8>());

//     this->pub_orb_features->publish(msg);

//     // if (times_called % 100 == 0)
//     // {
//     //     // std::printf("descriptors_left: \n");
//     //     // pretty_print_mat(descriptors_left);
//     //     // std::printf("descriptors_right: \n");
//     //     // pretty_print_mat(descriptors_right);

//     //     std::printf("keypoints_left: (size: %ld) \n",
//     keypoints_left.size());
//     //     for (auto &keypoint : keypoints_left)
//     //     {
//     //         pretty_print_keypoint(keypoint);
//     //     }
//     //     // std::printf("keypoints_right: \n");
//     //     // for (auto &keypoint : keypoints_right)
//     //     // {
//     //     //     pretty_print_keypoint(keypoint);
//     //     // }
//     // }
// }

// auto StereoInertialNode::pub_camera_pose_callback() -> void {
//     using namespace geometry_msgs::msg;
//     ORB_SLAM3::Tracking *tracker = this->orbslam3_system->get_tracker();
//     ORB_SLAM3::Frame &frame = tracker->mCurrentFrame;
//     Eigen::Vector3f camera_center = frame.GetCameraCenter();
//     Sophus::SE3<float> camera_pose = frame.GetPose();

//     auto msg = PoseStampedMsg();
//     std_msgs::msg::Header &header = msg.header;
//     header.frame_id = "map";
//     header.stamp = this->get_clock()->now();

//     Pose &pose = msg.pose;
//     Point &position = pose.position;
//     Quaternion &orientation = pose.orientation;

//     position.x = camera_center[0];
//     position.y = camera_center[1];
//     position.z = camera_center[2];

//     orientation.x = camera_pose.unit_quaternion().x();
//     orientation.y = camera_pose.unit_quaternion().y();
//     orientation.z = camera_pose.unit_quaternion().z();
//     orientation.w = camera_pose.unit_quaternion().w();

//     this->pub_camera_pose->publish(msg);
// }

// auto StereoInertialNode::pub_imu_callback() -> void
// {
//     auto lock = std::scoped_lock{this->mutex_imu};

//     while (!this->imubuf.empty())
//     {
//         this->pub_imu->publish(this->imubuf.front());
//         this->imubuf.pop();
//     }
// }

StereoInertialNode::~StereoInertialNode() {
    // Delete sync thread
    this->sync_thread->join();
    delete this->sync_thread;

    // Stop all threads
    this->orbslam3_system->Shutdown();

    // TODO: create a parameter to save the trajectory
    // Save camera trajectory
    this->orbslam3_system->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void StereoInertialNode::grab_imu(const ImuMsg::SharedPtr msg) {
    std::scoped_lock lock(this->mutex_imu);
    this->imubuf.push(msg);
}

void StereoInertialNode::grab_image_left(const ImageMsg::SharedPtr msgLeft) {
    std::scoped_lock lock(this->mutex_img_left);

    if (!this->img_left_buf.empty()) {
        this->img_left_buf.pop();
    }

    this->img_left_buf.push(msgLeft);
}

void StereoInertialNode::grab_image_right(const ImageMsg::SharedPtr msgRight) {
    std::scoped_lock lock(this->mutex_img_right);

    if (!this->img_right_buf.empty()) {
        this->img_right_buf.pop();
    }

    this->img_right_buf.push(msgRight);
}

auto StereoInertialNode::get_image(const ImageMsg::SharedPtr msg) -> cv::Mat {
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;

    try {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }

    if (cv_ptr->image.type() == 0) {
        return cv_ptr->image.clone();
    } else {
        std::cerr << "Error image type" << std::endl;
        return cv_ptr->image.clone();
    }
}

auto StereoInertialNode::sync_with_imu() -> void {
    // const double max_time_diff = 0.01; // 10ms
    const double max_time_diff = 0.5; // 500ms

    while (true) {
        // Declared here to avoid memory allocation in the loop
        cv::Mat img_left, img_right;
        double time_img_left = 0, time_img_right = 0;
        const bool left_images_available = !this->img_left_buf.empty();
        const bool right_images_available = !this->img_right_buf.empty();
        const bool images_available =
            left_images_available && right_images_available;
        const bool imu_measurements_available = !this->imubuf.empty();

        // if (!this->img_left_buf.empty() && !this->img_right_buf.empty() &&
        // !this->imubuf.empty())
        if (!imu_measurements_available || !images_available) {
            std::this_thread::sleep_for(1ms);
            continue;
        }

        time_img_left =
            utils::stamp2sec(this->img_left_buf.front()->header.stamp);
        time_img_right =
            utils::stamp2sec(this->img_right_buf.front()->header.stamp);
        // DEBUG_LOG(stderr, "time_img_left: %f, time_img_right: %f",
                  // time_img_left, time_img_right);

        {
            std::scoped_lock lock(this->mutex_img_right);
            while ((time_img_left - time_img_right) > max_time_diff &&
                   this->img_right_buf.size() > 1) {
                this->img_right_buf.pop();
                time_img_right =
                    utils::stamp2sec(this->img_right_buf.front()->header.stamp);
            }
        }

        {
            std::scoped_lock lock(this->mutex_img_left);
            while ((time_img_right - time_img_left) > max_time_diff &&
                   this->img_left_buf.size() > 1) {
                this->img_left_buf.pop();
                time_img_left =
                    utils::stamp2sec(this->img_left_buf.front()->header.stamp);
            }
        }

        const double dt = std::abs(time_img_left - time_img_right);
        if (dt > max_time_diff) {
            DEBUG_LOG(stderr, "dt: %f time_img_left: %f time_img_right: %f", dt,
                      time_img_left, time_img_right);
            DEBUG_LOG(stderr,
                      "to big time difference between left and right image");
            continue;
        }

        // if (time_img_left >
        // utils::stamp2sec(this->imubuf.back()->header.stamp))
        // {
        //     std::cerr << "time_img_left: " << time_img_left << " time_imu: "
        //     << utils::stamp2sec(this->imubuf.back()->header.stamp) <<
        //     std::endl; continue;
        // }

        // Get left and right images
        {
            std::scoped_lock lock(this->mutex_img_left);
            img_left = get_image(this->img_left_buf.front());
            this->img_left_buf.pop();
        }
        {
            std::scoped_lock lock(this->mutex_img_right);
            img_right = get_image(this->img_right_buf.front());
            this->img_right_buf.pop();
        }

        // Get imu measurements
        auto imu_measurements = std::vector<ORB_SLAM3::IMU::Point>();
        {
            std::scoped_lock lock(this->mutex_imu);
            if (!this->imubuf.empty()) {
                // Load imu measurements from buffer
                // imu_measurements.clear();
                while (!this->imubuf.empty() &&
                       utils::stamp2sec(this->imubuf.front()->header.stamp) <=
                           time_img_left) {
                    const double t =
                        utils::stamp2sec(this->imubuf.front()->header.stamp);
                    cv::Point3f acc(
                        this->imubuf.front()->linear_acceleration.x,
                        this->imubuf.front()->linear_acceleration.y,
                        this->imubuf.front()->linear_acceleration.z);
                    cv::Point3f gyr(this->imubuf.front()->angular_velocity.x,
                                    this->imubuf.front()->angular_velocity.y,
                                    this->imubuf.front()->angular_velocity.z);
                    imu_measurements.push_back(
                        ORB_SLAM3::IMU::Point(acc, gyr, t));
                    // DEBUG_LOG(stderr, "imu_measurements.size(): %lu",
                              // imu_measurements.size());
                    this->imubuf.pop();
                }
            }
        }

        if (this->apply_clahe) {
            this->clahe->apply(img_left, img_left);
            this->clahe->apply(img_right, img_right);
        }

        if (this->do_rectify) {
            cv::remap(img_left, img_left, M1l_, M2l_, cv::INTER_LINEAR);
            cv::remap(img_right, img_right, M1r_, M2r_, cv::INTER_LINEAR);
        }

        // Update ORB-SLAM3
        this->orbslam3_system->TrackStereo(img_left, img_right, time_img_left,
                                           imu_measurements);

        std::this_thread::sleep_for(1ms);
    }
}
