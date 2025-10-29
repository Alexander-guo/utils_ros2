# AFRL Utils

## Python Scripts

This folder contains python scripts that are used for various tasks that do not need to run as ros nodes.
The README.md file inside the folder contains more information on the scripts in that folder.

## ROS2 Package

### Build
```bash
mkdir -p ~/utils_ros2_ws/src 
cd ~/utils_ros2_ws/src
git clone https://github.com/Alexander-guo/utils_ros2.git
cd ..
colcon build --packages-select utils_ros2 --symlink-install
source ~/utils_ros2_ws/install/setup.bash # Or add this to ~/.bashrc to make it permanent
```

### Usage 

#### Extract Keyframes
To extract keyframes from a ros2bag, run the following command:
```bash
ros2 launch utils_ros2 write_keyframe_images.xml bag_file:=bag_file image_dir:=dir_to_save_images traj_file:=VIO_trajectory_file config_file:=camera config file
```

Note: By default, the keyframes are undistorted. The reason being COLMAP undistortion changes image size in an effort to maximize information from images.

The gopro config files are inside the ```config/gopro``` folder. 

Check the launch file for additional parameters.
