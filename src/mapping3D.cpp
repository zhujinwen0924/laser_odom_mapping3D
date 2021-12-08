//(Distributed Mobile Computing Lab, Beihang University)
/*
        2021.12.08 
        里程计叠加点云生成3D点云地图.接收里程计和点云，时间同步后从局部坐标系装换到全局坐标系.
        运行中按CTRL + C,结束程序并自动保存到mapData目录下面 
*/

#include <iostream>
#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/filters/voxel_grid.h>
#include <tf/LinearMath/Quaternion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <csignal>
#include <misc.h>
using namespace message_filters;


int  laser_count=0;
float vehicleX = 0;
float vehicleY = 0;
float vehicleZ = 0;

float quatX = 0;
float quatY = 0;
float quatZ = 0;
float quatW = 1;

ros::Publisher *pub3DMapPointer = NULL;
 std::string save_folder_path;
std::string save_file_name;
double downsample_voxel_size = 0.1;       

// ros::Publisher 3D_map_pub;
pcl::PointCloud<pcl::PointXYZI>::Ptr scanData(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr mapCloud(new pcl::PointCloud<pcl::PointXYZI>());
void CallBack(const nav_msgs::OdometryConstPtr& odom,const sensor_msgs::PointCloud2ConstPtr& scan)
{
    ROS_INFO("SyncCallback!\n");

    //检查时间戳
     float laser_stamp_nsec=scan->header.stamp.nsec;
     float laser_stamp_sec=scan->header.stamp.sec;

    float odom_stamp_nsec=odom->header.stamp.nsec; 
    float odom_stamp_sec=odom->header.stamp.sec; 
    float odom_stamp=(float)odom->header.stamp.sec+(float)odom->header.stamp.nsec/1000000000.0;
    float laser_stamp=(float)scan->header.stamp.sec+(float)scan->header.stamp.nsec/1000000000.0;

    // ROS_INFO("odom_stamp:%f\t%f",odom_stamp_sec,odom_stamp_nsec/1000000);
    // ROS_INFO("laser_stamp:%f\t%f",laser_stamp_sec,laser_stamp_nsec/1000000);
    // ROS_INFO("TIME error:%f ms\n",(laser_stamp-odom_stamp)*1000.0);

    //获取3D里程计
    vehicleX = odom->pose.pose.position.x;
     vehicleY = odom->pose.pose.position.y;
     vehicleZ = odom->pose.pose.position.z;
     quatX = odom->pose.pose.orientation.x;
     quatY = odom->pose.pose.orientation.y;
     quatZ = odom->pose.pose.orientation.z;
     quatW = odom->pose.pose.orientation.w;
  
    
    //将点云插入到全局地图
  scanData->clear();
  pcl::fromROSMsg(*scan, *scanData);

  int scanDataSize = scanData->points.size();                            //遍历所有点云并转换到map坐标系下面
  for (int i = 0; i < scanDataSize; i++) {
    float pointX1 = scanData->points[i].x;
    float pointY1 = scanData->points[i].y ;
    float pointZ1 = scanData->points[i].z ;          

    tf::Quaternion quat(quatX, quatY, quatZ, quatW);       //旋转
    tf::Matrix3x3 mat(quat);
    tf::Vector3 p(pointX1, pointY1, pointZ1);
    tf::Vector3 r = mat * p;

    float pointX3 =  r.getX() + vehicleX;
    float pointY3 =  r.getY() + vehicleY;
    float pointZ3 =  r.getZ() + vehicleZ;                              //平移至map坐标系原点

    scanData->points[i].x = pointX3;
    scanData->points[i].y = pointY3;
    scanData->points[i].z = pointZ3;
  }

  *mapCloud+=*scanData;

    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud (mapCloud); //设置需要滤波的点云
    sor.setLeafSize (downsample_voxel_size,downsample_voxel_size,downsample_voxel_size);  //设置滤波时创建的体素体积默认为0.1m3
    sor.filter (*mapCloud);   

  // publish 5Hz registered scan messages
//   sensor_msgs::PointCloud2 scanData2;
//   pcl::toROSMsg(*mapCloud, scanData2);
//   scanData2.header.stamp = ros::Time().now();
//   scanData2.header.frame_id = "/map";
//   pub3DMapPointer->publish(scanData2);     
  

  //当前帧点云添加到全局地图点云


}

void timerCallback(const ros::TimerEvent& event)
{
      //全局地图体素化下采样
      Timer  processTimer("publish");
      processTimer.Start();
    // pcl::VoxelGrid<pcl::PointXYZI> sor;
    // sor.setInputCloud (mapCloud); //设置需要滤波的点云
    // sor.setLeafSize (0.1f, 0.1f, 0.1f);  //设置滤波时创建的体素体积为0.1m3
    // sor.filter (*mapCloud);   

    /* 发布地图 */
  sensor_msgs::PointCloud2 scanData2;
  pcl::toROSMsg(*mapCloud, scanData2);
  scanData2.header.stamp = ros::Time().now();
  scanData2.header.frame_id = "/map";
  pub3DMapPointer->publish(scanData2);      
  processTimer.Stop(false);
  double dur=processTimer.GetDuration("ms");
  ROS_INFO("process and publish mapCloud,use time %.1f ms",dur);


}

void signalHander(int sigNum)
{
    ROS_INFO("USER Ctrl C pressed, save 3D map and exit now!");
    std::cout<<"folder_path_:"<<save_folder_path<<std::endl;
      std::stringstream filePath;
    filePath << save_folder_path << "/"<<save_file_name<<".pcd";
    std::cout<<"filePath:"<<filePath.str()<<endl;

    pcl::io::savePCDFile(filePath.str(), *mapCloud);
    ROS_INFO("3D map has saved in :");
    std::cout<<filePath.str()<<endl;
    exit(sigNum);
}

bool readParameters(ros::NodeHandle& nodeHandle_) {
  bool allParametersRead = true;
  allParametersRead = nodeHandle_.getParam("save_folder_path", save_folder_path) && allParametersRead;
  allParametersRead = nodeHandle_.getParam("save_file_name", save_file_name) && allParametersRead;
  nodeHandle_.getParam("downsample_voxel_size", downsample_voxel_size);

  if (!allParametersRead) {
    ROS_WARN(
        "Could not read all parameters. :\n"
    );
    return false;
  }

  return true;
}

int main ( int argc, char **argv )
{
    /***** 初始化ROS *****/
    ros::init ( argc, argv, "odom_laser3dMapping" );
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    /***** 加载参数 *****/
    if(!readParameters(private_nh)){
         ROS_WARN("readParameters error!");
    }

    /***** 初始化地图*****/


    /***** 初始Topic 和时间同步器*****/
    // typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry,sensor_msgs::PointCloud2> slamSyncPolicy;
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh,"/state_estimation",10);
    message_filters::Subscriber<sensor_msgs::PointCloud2> laser_sub(nh,"/sensor_scan",10);
    // Synchronizer<slamSyncPolicy>  sync(slamSyncPolicy(20),odom_sub,laser_sub);

      TimeSynchronizer<nav_msgs::Odometry, sensor_msgs::PointCloud2> sync(odom_sub, laser_sub, 10);         //时间同步很关键,否则叠加出来的地图会有残影
    sync.registerCallback(boost::bind(&CallBack,_1,_2));
    
    ros::Publisher  map3D_pub = nh.advertise<sensor_msgs::PointCloud2> ( "/3Dmap_output", 1 ,true);
    pub3DMapPointer=&map3D_pub;


    ros::Timer timer = nh.createTimer(ros::Duration(2.0), timerCallback);
    signal(SIGINT, signalHander);
    ros::spin();
    return  0;
}









