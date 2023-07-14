# AFRL Utils

## Python Scripts

This folder contains python scripts that are used for various tasks that do not need to run as ros nodes.
The README.md file inside the folder contains more information on the scripts in that folder.

### Build
```bash
mkdir -p ~/utils_ws/src 
cd ~/utils_ws/src
git clone git@github.com:AutonomousFieldRoboticsLab/utils.git
cd ..
catkin_make
```

### Update
```bash
cd ~/utils_ws/src/utils
git pull origin main
cd ../..
catkin_make
```
## ROS Nodes

### Extract Keyframes
To extract keyframes from a rosbag, run the following command:
```bash
roslaunch utils_ros write_keyframe_images.launch bag_file:=bag_file image_dir:=dir_to_save_images traj_file:=VIO_trajectory_file config_file:=camera config file
```

Note: By default, the keyframes are undistorted. The reason being COLMAP undistortion changes image size in an effort to maximize information from images.

The gopro config files are inside the config/gopro folder. 

Check the launch file for additional parameters.
