# laser_odom_mapping
## A simple implementation of laser_odom_mapping

input topic: state_estimation   pointcloud
output_topic: 3Dmap_output

## Demo

step1. Download the repository to your ROS workspace: catkin_ws/src

step2. Make：catkin_make

step3. Run:  roslaunch mapping3D mapping3D.launch

step4.  SCTL + C结束程序,自动保存地图到mapData目录下面.
