#include<iostream>
#include<string>
//#include<stdlib.h>
#include<stdio.h>
#include<sstream>
#include<vector>

#include<ros/ros.h>
#include<pcl/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include<sensor_msgs/PointCloud2.h>
#include<pcl/io/pcd_io.h>
#include<pcl/filters/voxel_grid.h>
using namespace std;

int main(int argc, char **argv)
{
	string topic, path, frame_id;
	int hz=5;
	
	ros::init(argc, argv, "cloudout");
	ros::NodeHandle nh;

	nh.param<string>("path", path, "/home/dengliyin/me/mycatkin_ws/src/cloudpublish/data/resultPointCloudFile.pcd");
	nh.param<string>("topic", topic, "/pointcloud/output");
	nh.param<int>("hz", hz, 5);
	nh.param<string>("frame_id", frame_id, "camera");

	ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2>(topic, 10);
	
	pcl::PointCloud<pcl::PointXYZRGBA> cloud;
	sensor_msgs::PointCloud2 output;
	pcl::io::loadPCDFile(path, cloud);

     	pcl::VoxelGrid<pcl::PointXYZRGBA> voxel_filter;
   	double resolution = 0.03;
   	voxel_filter.setLeafSize(resolution, resolution, resolution);       // resolution
   	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGBA>);
  	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr t1(new pcl::PointCloud<pcl::PointXYZRGBA>); 
  	*t1 = cloud;
   	voxel_filter.setInputCloud(t1);
   	voxel_filter.filter(*tmp);
    	tmp->swap(*t1);
    	cloud = *t1;

	pcl::toROSMsg(cloud, output);

	output.header.stamp = ros::Time(0);
	output.header.frame_id = frame_id;

	ros::Rate loop_rate(hz);
	while(ros::ok())
	{
		pcl_pub.publish(output);
		ros::spinOnce();
		loop_rate.sleep();

	}

	return 0;
}
