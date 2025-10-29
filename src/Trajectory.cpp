#include "Trajectory.h"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <fstream>
#include <Eigen/Core>
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

Trajectory::Trajectory(const std::string &traj_file)
    : traj_file_(traj_file)
{
}

bool Trajectory::loadTrajectory(char separator, bool skip_first_line, bool double_stamp)
{
    return loadTrajectory(traj_file_, separator, skip_first_line, double_stamp);
}

std::set<std::uint64_t> Trajectory::getTimestamps()
{
    return timestamps_;
}

std::vector<geometry_msgs::msg::Pose> Trajectory::getPoses()
{
    return poses_;
}

bool Trajectory::loadTrajectory(const std::string &traj_file, char separator, bool skip_first_line, bool double_stamp)
{

    stamp_to_indx_map_.clear();
    poses_.clear();
    timestamps_.clear();

    RCLCPP_INFO(rclcpp::get_logger("Trajectory"), "Parsing VIO trajectory data ....");
    std::ifstream fin(traj_file.c_str());

    if (!fin.is_open()) {
        RCLCPP_FATAL(rclcpp::get_logger("Trajectory"), "Cannot open file: %s", traj_file.c_str());
        return false;
    }

    // Skip the first line, containing the header.
    std::string line;
    if (skip_first_line)
        std::getline(fin, line);

    size_t index = 0;
    while (std::getline(fin, line))
    {
        std::uint64_t timestamp = 0;
        std::vector<double> data_raw;
        for (size_t i = 0u; i < 9; i++)
        {
            int idx = line.find_first_of(separator);
            if (i == 0u)
            {
                if (double_stamp)
                    timestamp = (std::uint64_t)(std::stold(line.substr(0, idx)) * 1000000000);
                else
                    timestamp = std::stoll(line.substr(0, idx));
            }
            else
            {
                data_raw.push_back(std::stod(line.substr(0, idx)));
            }
            line = line.substr(idx + 1);
        }

        geometry_msgs::msg::Point position;
        position.x = data_raw[0];
        position.y = data_raw[1];
        position.z = data_raw[2];

        // Quaternion x y z w.

        // Sanity check.
        Eigen::Vector4d q(data_raw[3], data_raw[4], data_raw[5], data_raw[6]);

        geometry_msgs::msg::Quaternion rot;
        rot.x = q(0);
        rot.y = q(1);
        rot.z = q(2);
        rot.w = q(3);

        geometry_msgs::msg::Pose pose;
        pose.position = position;
        pose.orientation = rot;

        stamp_to_indx_map_.insert({timestamp, index});
        poses_.push_back(pose);
        timestamps_.insert(timestamp);
        index++;
    }

    RCLCPP_INFO(rclcpp::get_logger("Trajectory"), "Added %zu poses to trajectory.", poses_.size());
    assert(timestamps_.size() == poses_.size());
    assert(timestamps_.size() == stamp_to_indx_map_.size());

    fin.close();
    return true;
}

bool Trajectory::getPose(const std::uint64_t &timestamp, geometry_msgs::msg::Pose &pose)
{
    if (stamp_to_indx_map_.find(timestamp) != stamp_to_indx_map_.end())
    {
        pose = poses_[stamp_to_indx_map_[timestamp]];
        return true;
    }
    else
    {
        return false;
    }
}
