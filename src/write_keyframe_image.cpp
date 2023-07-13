#include <ros/ros.h>
#include "Trajectory.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <experimental/filesystem>
#include "Utils.h"

namespace fs = std::experimental::filesystem;

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "write_images_from_list");
    ros::NodeHandle nh_private("~");

    std::string traj_file,
        bag_file, left_image_topic, right_image_topic, image_dir, config_file;

    ROS_FATAL_STREAM_COND(!nh_private.getParam("traj_file", traj_file),
                          "VIO keyframe trajectory file not set from params");
    ROS_FATAL_STREAM_COND(!nh_private.getParam("bag_file", bag_file),
                          "Bag file not set from params");
    ROS_FATAL_STREAM_COND(!nh_private.getParam("left_image_topic", left_image_topic),
                          "Image topic not set from params");
    ROS_FATAL_STREAM_COND(!nh_private.getParam("image_dir", image_dir),
                          "Output directory not set from params");
    ROS_FATAL_STREAM_COND(!nh_private.getParam("config_file", config_file),
                          "Config file not set from params");

    bool compressed = false;
    bool skip_first_line = false;
    bool stereo = false;
    double scale = 1.0;

    nh_private.param("compressed", compressed, compressed);
    nh_private.param("skip_first_line", skip_first_line, skip_first_line);
    nh_private.param("stereo", stereo, stereo);
    nh_private.param("scale", scale, scale);

    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if (!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to intrinsics" << std::endl;
    }

    cv::FileNode left_node = fsSettings["left"];
    cv::FileNode right_node = fsSettings["right"];

    cv::Mat left_distort = cv::Mat::zeros(4, 1, CV_64F);
    cv::Mat left_K = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat right_distort = cv::Mat::zeros(4, 1, CV_64F);
    cv::Mat right_K = cv::Mat::eye(3, 3, CV_64F);

    Utils::readIntrinsics(left_node, left_K, left_distort);
    Utils::readIntrinsics(right_node, right_K, right_distort);

    if (traj_file.empty() || bag_file.empty() || left_image_topic.empty() || image_dir.empty() || config_file.empty())
        exit(1);

    Trajectory traj(traj_file);
    traj.loadTrajectory(' ', skip_first_line, true);

    std::set<std::uint64_t> kf_stamps = traj.getTimestamps();

    std::string left_image_dir = image_dir + "/left";
    std::string right_image_dir = image_dir + "/right";

    if (!fs::exists(left_image_dir))
    {
        fs::create_directories(left_image_dir);
    }
    if (compressed)
        left_image_topic = left_image_topic + "/compressed";
    if (stereo)
    {
        ROS_FATAL_STREAM_COND(!nh_private.getParam("right_image_topic", right_image_topic),
                              "right image topic not set from params");

        if (!fs::exists(right_image_dir))
            fs::create_directories(right_image_dir);

        if (compressed)
            right_image_topic = right_image_topic + "/compressed";
    }

    rosbag::Bag bag;
    bag.open(bag_file, rosbag::bagmode::Read);

    rosbag::View view_left(bag, rosbag::TopicQuery(left_image_topic));
    rosbag::View view_right(bag, rosbag::TopicQuery(right_image_topic));

    cv::Mat image;
    std::int64_t stamp;

    rosbag::View::iterator it_left = view_left.begin();
    if (compressed)
    {
        sensor_msgs::CompressedImageConstPtr image_msg = it_left->instantiate<sensor_msgs::CompressedImage>();
        image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8)->image;
    }
    else
    {
        sensor_msgs::ImageConstPtr image_msg = it_left->instantiate<sensor_msgs::Image>();
        image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8)->image;
    }

    int original_width = image.size().width;
    int original_height = image.size().height;
    int new_width = static_cast<int>(original_width * scale);
    int new_height = static_cast<int>(original_height * scale);

    ROS_INFO_STREAM("Orignial height, width: " << image.size().height << ", " << image.size().width);
    ROS_INFO_STREAM("New height, width: " << new_height << ", " << new_width);

    cv::Mat left_map_x, left_map_y, right_map_x, right_map_y;
    cv::initUndistortRectifyMap(left_K, left_distort, cv::Mat(), left_K, cv::Size(original_width, original_height), CV_32FC1, left_map_x, left_map_y);
    cv::initUndistortRectifyMap(right_K, right_distort, cv::Mat(), right_K, cv::Size(original_width, original_height), CV_32FC1, right_map_x, right_map_y);

    size_t kf_images = 0;
    cv::Mat undistorted_image;
    for (auto it = view_left.begin(); it != view_left.end(); ++it)
    {
        if (compressed)
        {
            sensor_msgs::CompressedImageConstPtr image_msg = it->instantiate<sensor_msgs::CompressedImage>();
            image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8)->image;
            stamp = image_msg->header.stamp.toNSec();
        }
        else
        {
            sensor_msgs::ImageConstPtr image_msg = it->instantiate<sensor_msgs::Image>();
            image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8)->image;
            stamp = image_msg->header.stamp.toNSec();
        }
        std::string filename = left_image_dir + "/" + std::to_string(stamp) + ".png";

        // cv::imshow("image", image);
        // cv::waitKey(1);

        if (kf_stamps.find(stamp) != kf_stamps.end())
        {
            cv::remap(image, undistorted_image, left_map_x, left_map_y, cv::INTER_LINEAR);
            if (scale != 1.0)
                cv::resize(undistorted_image, undistorted_image, cv::Size(new_width, new_height));
            cv::imwrite(filename, undistorted_image);
            kf_images++;
        }
    }

    for (auto it = view_right.begin(); it != view_right.end(); ++it)
    {
        if (compressed)
        {
            sensor_msgs::CompressedImageConstPtr image_msg = it->instantiate<sensor_msgs::CompressedImage>();
            image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8)->image;
            stamp = image_msg->header.stamp.toNSec();
        }
        else
        {
            sensor_msgs::ImageConstPtr image_msg = it->instantiate<sensor_msgs::Image>();
            image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8)->image;
            stamp = image_msg->header.stamp.toNSec();
        }
        std::string filename = right_image_dir + "/" + std::to_string(stamp) + ".png";
        if (kf_stamps.find(stamp) != kf_stamps.end())
        {
            cv::remap(image, undistorted_image, right_map_x, right_map_y, cv::INTER_LINEAR);
            if (scale != 1.0)
                cv::resize(undistorted_image, undistorted_image, cv::Size(new_width, new_height));
            cv::imwrite(filename, undistorted_image);
        }
    }

    ROS_INFO_STREAM("Wrote " << kf_images << "/" << view_left.size() << " images");
    bag.close();

    return 0;
}
