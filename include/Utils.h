#pragma once

#include <iostream>
#include <vector>
#include <tf2/LinearMath/Quaternion.h>
#include <opencv2/core/core.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
namespace Utils
{

    inline void printTf2Quat(const tf2::Quaternion &q, const std::string &info = "")
    {

        std::cout << info << q.x() << "\t" << q.y() << "\t" << q.z() << "\t" << q.w() << "\n";
    }

    bool getImageStamps(const std::string &folder_path,
                        std::vector<std::uint64_t> &time_stamps);

    bool getStampsFromTrajectory(const std::string &trajectory_path,
                                 std::vector<std::uint64_t> &time_stamps,
                                 bool skip_first_line = false);
    const cv::Mat readRosImage(const sensor_msgs::ImageConstPtr &img_msg, bool grayscale = true);
    const cv::Mat readCompressedRosImage(const sensor_msgs::CompressedImageConstPtr &img_msg, bool grayscale = true);

    void readIntrinsics(cv::FileNode &cam_node, cv::Mat &K, cv::Mat &distortion_coeffs);

} // namespace Utils
