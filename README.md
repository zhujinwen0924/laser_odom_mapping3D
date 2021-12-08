# Occupancy Grid Mapping
## A simple implementation of occupancy grid mapping.

input topic: state_estimation   pointcloud
output_topic: 3Dmap_output

## Demo

step1. Download the repository to your ROS workspace: catkin_ws/src

step2. Make：catkin_make

step3. Run: roslaunch occ_grid_mapping mapping.launch

step4.  SCTL + C结束程序,自动保存地图到mapData目录下面.
