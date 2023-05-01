#pragma once

#include <iostream>
#include <cassert>
#include "opencv2/opencv.hpp"

void pretty_print_mat(const cv::Mat &mat, const int precision = 2)
{
    // Set the desired precision for printing floating-point numbers
    assert(precision >= 0);
    std::printf("shape: (%d, %d)\n", mat.rows, mat.cols);
    // Iterate through the rows and columns of the matrix
    for (int i = 0; i < mat.rows; ++i)
    {
        for (int j = 0; j < mat.cols; ++j)
        {
            // Handle different data types in the cv::Mat
            switch (mat.type())
            {
            case CV_8U:
                std::printf("%*d\t", precision, static_cast<int>(mat.at<uchar>(i, j)));
                break;
            case CV_8S:
                std::printf("%*d\t", precision, static_cast<int>(mat.at<schar>(i, j)));
                break;
            case CV_16U:
                std::printf("%*u\t", precision, mat.at<ushort>(i, j));
                break;
            case CV_16S:
                std::printf("%*d\t", precision, mat.at<short>(i, j));
                break;
            case CV_32S:
                std::printf("%*d\t", precision, mat.at<int>(i, j));
                break;
            case CV_32F:
                std::printf("%*.*f\t", precision + 3, precision, mat.at<float>(i, j));
                break;
            case CV_64F:
                std::printf("%*.*f\t", precision + 3, precision, mat.at<double>(i, j));
                break;
            default:
                std::cout << "Unsupported cv::Mat data type.";
                return;
            }
        }
        std::printf("\n");
    }
}

void pretty_print_keypoint(const cv::KeyPoint &keypoint)
{
    // Set the desired precision for printing floating-point numbers
    int precision = 2;
    std::cout.precision(precision);
    std::cout << std::fixed;

    // Print keypoint properties
    std::cout << "KeyPoint (x: " << keypoint.pt.x
              << ", y: " << keypoint.pt.y
              << ", size: " << keypoint.size
              << ", angle: " << keypoint.angle
              << ", response: " << keypoint.response
              << ", octave: " << keypoint.octave
              << ", class_id: " << keypoint.class_id
              << ")" << std::endl;
}
