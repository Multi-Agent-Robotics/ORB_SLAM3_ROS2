#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <exception>
#include "rclcpp/rclcpp.hpp"

namespace utils
{
    auto stamp2sec(const builtin_interfaces::msg::Time &stamp) -> double
    {
        // double seconds = stamp.sec + (stamp.nanosec * pow(10, -9));
        double seconds = stamp.sec + (stamp.nanosec * 1e-9);
        return seconds;
    }

    auto hr(const char c, unsigned int n = 80) -> void
    {
        std::cout << std::string(n, c) << std::endl;
    }

    // A utility function that takes a variadic number of strings
    // and concatenates them into a single string formatted as a ros2 topic path
    template <typename... Args>
    auto format_topic_path(Args... args) -> std::string
    {
        const std::vector<std::string> topic_path = {args...};
        if (!(topic_path.size() > 0))
        {
            throw std::invalid_argument("format_topic_path() requires at least one argument");
        }
        std::string topic_path_str = "";
        if (topic_path.size() == 1)
        {
            topic_path_str = topic_path[0];
        }
        else
        {
            for (std::size_t i = 0; i < topic_path.size() - 1; i++)
            {
                topic_path_str += topic_path[i] + "/";
            }
            topic_path_str += topic_path.back();
        }
        return topic_path_str;
    }

} // namespace utils
