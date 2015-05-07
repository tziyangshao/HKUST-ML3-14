#include "KeyFrameDisplay.h"

#include <sstream>
#include <fstream>
#include <iostream>
#include "ros/ros.h"
#include <stdint.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

// PCL specific includes
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

class pc_filter{
    public:
	pc_filter();
	~pc_filter();	
	void pcCallback(const sensor_msgs::PointCloud2ConstPtr & cloud2);
	
    private:
	
	ros::NodeHandle _slaveRobot;
	ros::Subscriber _pcSub;
	ros::Publisher _pcPub;
};

