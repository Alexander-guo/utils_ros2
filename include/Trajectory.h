#include <string>
#include <geometry_msgs/Pose.h>
#include <unordered_map>
#include <set>

class Trajectory
{
    public:
        Trajectory(std::string &traj_file);
        ~Trajectory() = default;

        bool loadTrajectory(char separator = ' ', bool skip_first_line = false, bool double_stamp = true);
        bool loadTrajectory(std::string &traj_file, char separator = ' ', bool skip_first_line = false, bool double_stamp = true);

        bool getPose(const std::uint64_t &timestamp, geometry_msgs::Pose &pose);
        
        std::set<std::uint64_t> getTimestamps();
        std::vector<geometry_msgs::Pose> getPoses();

    private:
        std::string traj_file_;
        std::unordered_map<std::uint64_t, size_t> stamp_to_indx_map_;
        std::set<std::uint64_t> timestamps_;
        std::vector<geometry_msgs::Pose> poses_;
};