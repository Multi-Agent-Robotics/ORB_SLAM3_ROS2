#include "stereo-inertial-node.hpp"

#include <opencv2/core/core.hpp>

#include <chrono>
#include <cstdio>

using namespace std::chrono_literals;
using std::placeholders::_1;



StereoInertialNode::StereoInertialNode(ORB_SLAM3::System *SLAM, const string& settings_filepath, bool do_rectify, bool do_equalize) :
    Node("orbslam3"),
    this->orbslam3_system(SLAM),
    this->do_rectify(do_rectify),
    this->do_equalize(do_equalize),
    this->apply_clahe(do_equalize)
{
    if (this->do_rectify)
    {
        // Load settings related to stereo calibration
        auto settings cv::FileStorage(settings_filepath, cv::FileStorage::READ);
        if (!settings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            std::exit(1);
        }

        cv::Mat left_K, right_K, left_P, right_P, left_R, right_R, left_D, right_D;
        
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

        if (left_K.empty() || right_K.empty() || left_P.empty() || right_P.empty() || left_R.empty() || right_R.empty() || left_D.empty() || right_D.empty() ||
            rows_l == 0 || rows_r == 0 || cols_l == 0 || cols_r == 0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            std::exit(1);
        }

        cv::initUndistortRectifyMap(left_K, left_D, left_R, left_P.rowRange(0, 3).colRange(0, 3), cv::Size(cols_l, rows_l), CV_32F, M1l_, M2l_);
        cv::initUndistortRectifyMap(right_K, right_D, right_R, right_P.rowRange(0, 3).colRange(0, 3), cv::Size(cols_r, rows_r), CV_32F, M1r_, M2r_);
    }

    this->sub_imu = this->create_subscription<ImuMsg>("imu", 1000, std::bind(&StereoInertialNode::grab_imu, this, _1));
    this->sub_img_left = this->create_subscription<ImageMsg>("camera/left", 100, std::bind(&StereoInertialNode::grab_image_left, this, _1));
    this->sub_img_right = this->create_subscription<ImageMsg>("camera/right", 100, std::bind(&StereoInertialNode::grab_image_right, this, _1));

    this->sync_thread = new std::thread(&StereoInertialNode::sync_with_imu, this);

    // Publishers 
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    this->pub_imu = this->create_publisher<ImuMsg>("imu_corrected", qos);
    this->pub_imu_timer = this->create_wall_timer(
        100ms,
        std::bind(&StereoInertialNode::pub_imu_callback, this)
    );
}

auto StereoInertialNode::pub_imu_callback() -> void
{
    std::scoped_lock<std::mutex> lock(this->mutex_imu);

    while (!this->imubuf.empty())
    {
        this->pub_imu->publish(this->imubuf.front());
        this->imubuf.pop();
    }
}


StereoInertialNode::~StereoInertialNode()
{
    // Delete sync thread
    this->sync_thread->join();
    delete this->sync_thread;

    // Stop all threads
    this->orbslam3_system->Shutdown();

    // TODO: create a parameter to save the trajectory
    // Save camera trajectory
    this->orbslam3_system->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void StereoInertialNode::grab_imu(const ImuMsg::SharedPtr msg)
{
    std::scoped_lock<std::mutex> lock(this->mutex_imu);
    this->imubuf.push(msg);
}

void StereoInertialNode::grab_image_left(const ImageMsg::SharedPtr msgLeft)
{
    std::scoped_lock<std::mutex> lock(this->mutex_img_left);

    if (!this->img_left_buf.empty()) {
        this->img_left_buf.pop();
    }

    this->img_left_buf.push(msgLeft);
}

void StereoInertialNode::grab_image_right(const ImageMsg::SharedPtr msgRight)
{
    std::scoped_lock<std::mutex> lock(this->mutex_img_right);

    if (!this->img_right_buf.empty()) {
        this->img_right_buf.pop();
    }

    this->img_right_buf.push(msgRight);
}

auto StereoInertialNode::get_image(const ImageMsg::SharedPtr msg) -> cv::Mat
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }

    if (cv_ptr->image.type() == 0)
    {
        return cv_ptr->image.clone();
    }
    else
    {
        std::cerr << "Error image type" << std::endl;
        return cv_ptr->image.clone();
    }
}

auto StereoInertialNode::sync_with_imu() -> void
{
    const double max_time_diff = 0.01; // 10ms

    while (true)
    {
        // Declared here to avoid memory allocation in the loop
        cv::Mat img_left, img_right;
        double time_img_left = 0, time_img_right = 0;

        if (!this->img_left_buf.empty() && !this->img_right_buf.empty() && !this->imubuf.empty())
        {
            time_img_left = Utility::StampToSec(this->img_left_buf.front()->header.stamp);
            time_img_right = Utility::StampToSec(this->img_right_buf.front()->header.stamp);

            {
                std::scoped_lock<std::mutex> lock(this->mutex_img_right);
                while ((time_img_left - time_img_right) > max_time_diff && this->img_right_buf.size() > 1)
                {
                    this->img_right_buf.pop();
                    time_img_right = Utility::StampToSec(this->img_right_buf.front()->header.stamp);
                }
            }

            {
                std::scoped_lock<std::mutex> lock(this->mutex_img_left);
                while ((time_img_right - time_img_left) > max_time_diff && this->img_left_buf.size() > 1)
                {
                    this->img_left_buf.pop();
                    time_img_left = Utility::StampToSec(this->img_left_buf.front()->header.stamp);
                }
            }

            const double dt = std::abs(time_img_left - time_img_right);
            if (dt > max_time_diff)
            {
                std::cerr << "big time difference between left and right image" << std::endl;
                continue;
            }

            if (time_img_left > Utility::StampToSec(this->imubuf.back()->header.stamp)) {
                continue;
            }

            {
                std::scoped_lock<std::mutex> lock(this->mutex_img_left);
                img_left = get_image(this->img_left_buf.front());
                this->img_left_buf.pop();
            }
            {
                std::scoped_lock<std::mutex> lock(this->mutex_img_right);
                img_right = get_image(this->img_right_buf.front());
                this->img_right_buf.pop();
            }

            auto imu_measurements = std::vector<ORB_SLAM3::IMU::Point>();
            {
                std::scoped_lock<std::mutex> lock(this->mutex_imu);
                if (!this->imubuf.empty())
                {
                    // Load imu measurements from buffer
                    imu_measurements.clear();
                    while (!this->imubuf.empty() && Utility::StampToSec(this->imubuf.front()->header.stamp) <= time_img_left)
                    {
                        const double t = Utility::StampToSec(this->imubuf.front()->header.stamp);
                        cv::Point3f acc(this->imubuf.front()->linear_acceleration.x, this->imubuf.front()->linear_acceleration.y, this->imubuf.front()->linear_acceleration.z);
                        cv::Point3f gyr(this->imubuf.front()->angular_velocity.x, this->imubuf.front()->angular_velocity.y, this->imubuf.front()->angular_velocity.z);
                        imu_measurements.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
                        this->imubuf.pop();
                    }
                }
            }
            
            if (this->apply_clahe)
            {
                this->clahe->apply(img_left, img_left);
                this->clahe->apply(img_right, img_right);
            }

            if (this->do_rectify)
            {
                cv::remap(img_left, img_left, M1l_, M2l_, cv::INTER_LINEAR);
                cv::remap(img_right, img_right, M1r_, M2r_, cv::INTER_LINEAR);
            }

            this->orbslam3_system->TrackStereo(img_left, img_right, time_img_left, imu_measurements);

            std::this_thread::sleep_for(1ms);
        }
    }
}
