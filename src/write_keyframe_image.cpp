#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"
#include "Trajectory.h"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include <opencv2/opencv.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <filesystem>
#include "Utils.h"

namespace fs = std::filesystem;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("write_images_from_list");

    std::string traj_file,
        bag_file, left_image_topic, right_image_topic, image_dir, config_file;

    node->declare_parameter("traj_file", "");
    node->declare_parameter("bag_file", "");
    node->declare_parameter("left_image_topic", "");
    node->declare_parameter("image_dir", "");
    node->declare_parameter("config_file", "");

    traj_file = node->get_parameter("traj_file").as_string();
    bag_file = node->get_parameter("bag_file").as_string();
    left_image_topic = node->get_parameter("left_image_topic").as_string();
    image_dir = node->get_parameter("image_dir").as_string();
    config_file = node->get_parameter("config_file").as_string();

    if (traj_file.empty()) {
        RCLCPP_FATAL(node->get_logger(), "VIO keyframe trajectory file not set from params");
        return 1;
    }
    if (bag_file.empty()) {
        RCLCPP_FATAL(node->get_logger(), "Bag file not set from params");
        return 1;
    }
    if (left_image_topic.empty()) {
        RCLCPP_FATAL(node->get_logger(), "Image topic not set from params");
        return 1;
    }
    if (image_dir.empty()) {
        RCLCPP_FATAL(node->get_logger(), "Output directory not set from params");
        return 1;
    }
    if (config_file.empty()) {
        RCLCPP_FATAL(node->get_logger(), "Config file not set from params");
        return 1;
    }

    bool compressed = false;
    bool skip_first_line = false;
    bool stereo = false;
    double scale = 1.0;

    node->declare_parameter("compressed", false);
    node->declare_parameter("skip_first_line", false);
    node->declare_parameter("stereo", false);
    node->declare_parameter("scale", 1.0);

    compressed = node->get_parameter("compressed").as_bool();
    skip_first_line = node->get_parameter("skip_first_line").as_bool();
    stereo = node->get_parameter("stereo").as_bool();
    scale = node->get_parameter("scale").as_double();

    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if (!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to intrinsics" << std::endl;
    }
    cv::FileNode left_node, right_node;
    if (stereo)
    {
        cv::FileNode left_node = fsSettings["left"];
        cv::FileNode right_node = fsSettings["right"];
    }
    else
    {
        left_node = fsSettings.root();
    }

    cv::Mat left_distort = cv::Mat::zeros(4, 1, CV_64F);
    cv::Mat left_K = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat right_distort = cv::Mat::zeros(4, 1, CV_64F);
    cv::Mat right_K = cv::Mat::eye(3, 3, CV_64F);

    if (stereo)
    {
        Utils::readIntrinsics(right_node, right_K, right_distort);
    }

    Utils::readIntrinsics(left_node, left_K, left_distort);

    if (traj_file.empty() || bag_file.empty() || left_image_topic.empty() || image_dir.empty() || config_file.empty())
        exit(1);

    Trajectory traj(traj_file);
    traj.loadTrajectory(' ', skip_first_line, true);

    std::set<std::uint64_t> kf_stamps = traj.getTimestamps();
    std::string left_image_dir, right_image_dir;

    if (stereo)
    {
        left_image_dir = image_dir + "/left";
        right_image_dir = image_dir + "/right";
    }
    else
    {
        left_image_dir = image_dir;
    }

    if (!fs::exists(left_image_dir))
    {
        fs::create_directories(left_image_dir);
    }
    if (compressed)
        left_image_topic = left_image_topic + "/compressed";
    if (stereo)
    {
        node->declare_parameter("right_image_topic", "");
        right_image_topic = node->get_parameter("right_image_topic").as_string();
        if (right_image_topic.empty()) {
            RCLCPP_FATAL(node->get_logger(), "right image topic not set from params");
            return 1;
        }

        if (!fs::exists(right_image_dir))
            fs::create_directories(right_image_dir);

        if (compressed)
            right_image_topic = right_image_topic + "/compressed";
    }

    rosbag2_cpp::Reader reader;
    reader.open(bag_file);

    cv::Mat image;
    std::int64_t stamp;
    bool first_message = true;

    while (reader.has_next()) {
        auto bag_message = reader.read_next();
        if (bag_message->topic_name != left_image_topic) {
            continue;
        }

        if (first_message) {
            if (compressed)
            {
                auto image_msg = std::make_shared<sensor_msgs::msg::CompressedImage>();
                rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
                rclcpp::Serialization<sensor_msgs::msg::CompressedImage> serialization;
                serialization.deserialize_message(&serialized_msg, image_msg.get());
                image = cv_bridge::toCvCopy(image_msg, "bgr8")->image;
            }
            else
            {
                auto image_msg = std::make_shared<sensor_msgs::msg::Image>();
                rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
                rclcpp::Serialization<sensor_msgs::msg::Image> serialization;
                serialization.deserialize_message(&serialized_msg, image_msg.get());
                image = cv_bridge::toCvCopy(image_msg, "bgr8")->image;
            }
            first_message = false;
            break;
        }
    }

    // Reset reader position
    reader.close();
    reader.open(bag_file);

    int original_width = image.size().width;
    int original_height = image.size().height;
    int new_width = static_cast<int>(original_width * scale);
    int new_height = static_cast<int>(original_height * scale);

    RCLCPP_INFO_STREAM(node->get_logger(), "Original height, width: "
                   << image.size().height << ", " << image.size().width);
    RCLCPP_INFO_STREAM(node->get_logger(), "New height, width: " << new_height << ", " << new_width);

    cv::Mat left_map_x, left_map_y, right_map_x, right_map_y;
    cv::initUndistortRectifyMap(left_K, left_distort, cv::Mat(), left_K, cv::Size(original_width, original_height), CV_32FC1, left_map_x, left_map_y);
    if (stereo)
        cv::initUndistortRectifyMap(right_K, right_distort, cv::Mat(), right_K, cv::Size(original_width, original_height), CV_32FC1, right_map_x, right_map_y);

    size_t kf_images = 0;
    size_t total_images = 0;
    cv::Mat undistorted_image;
    
    while (reader.has_next()) {
        auto bag_message = reader.read_next();
        if (bag_message->topic_name != left_image_topic) {
            continue;
        }
        total_images++;

        if (compressed)
        {
            auto image_msg = std::make_shared<sensor_msgs::msg::CompressedImage>();
            rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
            rclcpp::Serialization<sensor_msgs::msg::CompressedImage> serialization;
            serialization.deserialize_message(&serialized_msg, image_msg.get());
            image = cv_bridge::toCvCopy(image_msg, "bgr8")->image;
            stamp = image_msg->header.stamp.nanosec + image_msg->header.stamp.sec * 1000000000ULL;
        }
        else
        {
            auto image_msg = std::make_shared<sensor_msgs::msg::Image>();
            rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
            rclcpp::Serialization<sensor_msgs::msg::Image> serialization;
            serialization.deserialize_message(&serialized_msg, image_msg.get());
            image = cv_bridge::toCvCopy(image_msg, "bgr8")->image;
            stamp = image_msg->header.stamp.nanosec + image_msg->header.stamp.sec * 1000000000ULL;
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

    if (stereo)
    {
        reader.close();
        reader.open(bag_file);
        while (reader.has_next())
        {
            auto bag_message = reader.read_next();
            if (bag_message->topic_name != right_image_topic) {
                continue;
            }

            if (compressed)
            {
                auto image_msg = std::make_shared<sensor_msgs::msg::CompressedImage>();
                rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
                rclcpp::Serialization<sensor_msgs::msg::CompressedImage> serialization;
                serialization.deserialize_message(&serialized_msg, image_msg.get());
                image = cv_bridge::toCvCopy(image_msg, "bgr8")->image;
                stamp = image_msg->header.stamp.nanosec + image_msg->header.stamp.sec * 1000000000ULL;
            }
            else
            {
                auto image_msg = std::make_shared<sensor_msgs::msg::Image>();
                rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
                rclcpp::Serialization<sensor_msgs::msg::Image> serialization;
                serialization.deserialize_message(&serialized_msg, image_msg.get());
                image = cv_bridge::toCvCopy(image_msg, "bgr8")->image;
                stamp = image_msg->header.stamp.nanosec + image_msg->header.stamp.sec * 1000000000ULL;
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
    }

    RCLCPP_INFO(node->get_logger(), "Wrote %zu/%zu images", kf_images, total_images);
    reader.close();
    rclcpp::shutdown();

    return 0;
}
