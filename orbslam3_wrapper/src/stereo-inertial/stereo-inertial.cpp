#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <string>
#include <cstdio>

#include "rclcpp/rclcpp.hpp"
#include "stereo-inertial-node.hpp"

// #include "utils.hpp"

#include "orbslam3/System.h" // ORB_SLAM3

using std::string;

namespace ros2 = rclcpp;

auto main(int argc, char **argv) -> int
{
    if (argc < 4)
    {
        std::cerr << "Usage: ros2 run orbslam stereo path_to_vocabulary path_to_settings do_rectify [do_equalize]" << std::endl;
        ros2::shutdown();
        return 1;
    }

    const string path_to_vocabulary = string(argv[1]);
    const string path_to_settings = string(argv[2]);

    auto str2bool = [](const string &s) -> bool
    {
        return s == "true" ? true : false;
    };

    const string arg3 = string(argv[3]);

    const bool do_rectify = str2bool(arg3);
    const string arg4 = argc == 5 ? string(argv[4]) : "false";
    const bool do_equalize = str2bool(arg4);

    std::printf("path_to_vocabulary = %s\n", path_to_vocabulary.c_str());
    std::printf("path_to_settings   = %s\n", path_to_settings.c_str());
    std::printf("do_rectify         = %d\n", do_rectify);
    std::printf("do_equalize        = %d\n", do_equalize);

    {
        bool failed = false;

        // Check settings file and vocabulary file exists
        auto file_exists = [](const std::string &path)
        {
            auto f = std::ifstream(path.c_str());
            bool exists = f.good();
            f.close();
            return exists;
        };

        if (!file_exists(path_to_vocabulary))
        {
            std::printf("Vocabulary file does not exist at: %s\n", path_to_vocabulary.c_str());
            failed = true;
        }

        if (!file_exists(path_to_settings))
        {
            std::printf("Settings file does not exist at: %s\n", path_to_settings.c_str());
            failed = true;
        }

        if (failed)
        {
            ros2::shutdown();
            return 1;
        }
    }

    ros2::init(argc, argv);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    const bool use_viewer = true;
    orbslam3::System orbslam3_system(path_to_vocabulary, path_to_settings, orbslam3::System::IMU_STEREO, use_viewer);

    auto node = std::make_shared<StereoInertialNode>("orbslam3", &orbslam3_system, path_to_settings, do_rectify, do_equalize);
    // utils::hr('=', 80);

    ros2::spin(node);
    ros2::shutdown();

    return 0;
}
