# Laserscan Odom mapping
## A simple implementation of Laserscan Odom mapping

input topic: state_estimation   pointcloud

output_topic: 3Dmap_output

## Demo

step1. Download the repository to your ROS workspace: catkin_ws/src

step2. Make：catkin_make

step3. Run: roslaunch mapping3D mapping3D.launch

step4.  SCTL + C结束程序,自动保存地图到mapData目录下面.

![图片](https://user-images.githubusercontent.com/57209631/145226786-018401cd-8731-4877-a442-8ab9e5cf6898.png)

rqt_graph:
![图片](https://user-images.githubusercontent.com/57209631/145227064-51b30b34-22c1-4698-8b16-8bc0c11dae28.png)
