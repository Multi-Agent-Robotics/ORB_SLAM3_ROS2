#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <string>
#include <cstdio>

#include "rclcpp/rclcpp.hpp"
#include "stereo-inertial-node.hpp"

#include "System.h" // ORB_SLAM3

using std::string;

auto hr(const char c, unsigned int n = 80) -> void {
    std::cout << std::string(n, c) << std::endl;
}


int main(int argc, char **argv) {

    if(argc < 4)
    {
        std::cerr << "\nUsage: ros2 run orbslam stereo path_to_vocabulary path_to_settings do_rectify [do_equalize]" << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    
    const string path_to_vocabulary = string(argv[1]);
    const string path_to_settings = string(argv[2]);
    const string do_rectify = string(argv[3]);
    const string do_equalize = 
        argc == 5
        ? string(argv[4])
        : "false";

    std::printf("path_to_vocabulary = %s\n", path_to_vocabulary.c_str());
    std::printf("path_to_settings = %s\n", path_to_settings.c_str());
    std::printf("do_rectify = %s\n", do_rectify.c_str());
    std::printf("do_equalize = %s\n", do_equalize.c_str());

    bool failed = false;

    // Check settings file and vocabulary file exists
    auto file_exists = [](const std::string& path) {
        auto f = std::ifstream(path.c_str());
        bool exists = f.good();
        f.close();
        return exists;
    }

    if (!file_exists(path_to_vocabulary)) {
        std::printf("Vocabulary file does not exist at: %s\n", path_to_vocabulary.c_str());
        failed = true;
    }

    if (!file_exists(path_to_settings)) {
        std::printf("Settings file does not exist at: %s\n", path_to_settings.c_str());
        failed = true;
    }

    // Check rectify and equalize are valid    
    if(do_rectify != "true" && do_rectify != "false")
    {
        std::printf("do_rectify must be true or false, but is %s\n", do_rectify.c_str());
        failed = true;
    }

    if(do_equalize != "true" && do_equalize != "false")
    {
        std::printf("do_equalize must be true or false, but is %s\n", do_equalize.c_str());
        failed = true;
    }

    if (failed) {
        rclcpp::shutdown();
        return 1;
    }



    rclcpp::init(argc, argv);

    // malloc error using new.. try shared ptr
    // Create SLAM system. It initializes all system threads and gets ready to process frames.

    const bool use_viewer = true;
    // ORB_SLAM3::System pSLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_STEREO, visualization);
    ORB_SLAM3::System orbslam3_system(path_to_vocabulary, path_to_settings, ORB_SLAM3::System::IMU_STEREO, use_viewer);

    // auto node = std::make_shared<StereoInertialNode>(&orbslam3_system, path_to_settings, argv[3], argv[4]);
    auto node = std::make_shared<StereoInertialNode>(&orbslam3_system, path_to_settings, do_rectify, do_equalize);
    hr('=', 80)

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
