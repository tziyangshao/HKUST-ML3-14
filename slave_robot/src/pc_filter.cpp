#include "pc_filter.h"
#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h> //I believe you were using pcl/ros/conversion.h
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/random_sample.h>


pc_filter::pc_filter(){
	_pcSub = _slaveRobot.subscribe("/slave_robot2/pointcloud2", 5, &pc_filter::pcCallback, this);
	_pcPub = _slaveRobot.advertise<sensor_msgs::PointCloud2>("/slave_robot2/pointcloudFiltered", 5);
}
pc_filter::~pc_filter(){
	//destructor to release the memory
}

void pc_filter::pcCallback(const sensor_msgs::PointCloud2ConstPtr & cloud_msg)
{ 
// Container for original & filtered data
  //int numberSampled = (cloud_msg.width)/2;

  ROS_INFO("output of numbers %f",cloud_msg->width);
  int numberSampled = 200;
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  // Perform the actual filtering
  
  pcl::RandomSample<pcl::PCLPointCloud2> ransamp;
  ransamp.setSample (numberSampled);
  ransamp.setSeed(rand());
  ransamp.filter(cloud_filtered);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(cloud_filtered, output);
  output.header.frame_id="world";
  ROS_INFO("output of numbers %f",output.width);
  // Publish the data
  _pcPub.publish (output);
}

