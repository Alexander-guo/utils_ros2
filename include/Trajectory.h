#include <string>
#include <vector>
#include <unordered_map>
#include <set>
#include <cstdint>
#include "geometry_msgs/msg/pose.hpp"

class Trajectory
{
    public:
        Trajectory(const std::string &traj_file);
        ~Trajectory() = default;

        bool loadTrajectory(char separator = ' ', bool skip_first_line = false, bool double_stamp = true);
        bool loadTrajectory(const std::string &traj_file, char separator = ' ', bool skip_first_line = false, bool double_stamp = true);

        bool getPose(const std::uint64_t &timestamp, geometry_msgs::msg::Pose &pose);

        std::set<std::uint64_t> getTimestamps();
        std::vector<geometry_msgs::msg::Pose> getPoses();

    private:
        std::string traj_file_;
        std::unordered_map<std::uint64_t, size_t> stamp_to_indx_map_;
        std::set<std::uint64_t> timestamps_;
        std::vector<geometry_msgs::msg::Pose> poses_;
};