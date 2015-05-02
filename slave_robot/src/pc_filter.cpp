#include "pc_filter.h"

using namespace std; 
using namespace sensor_msgs;


pc_filter::pc_filter(){
	_pcSub = _slaveRobot.subscribe("/slave_robot/pointcloud", 5, &pc_filter::pcCallback, this);
	_pcPub = _slaveRobot.advertise<sensor_msgs::PointCloud2>("/slave_robot/pointcloudFiltered", 5);
}
pc_filter::~pc_filter(){
	//destructor to release the memory
}

void pc_filter::pcCallback(const sensor_msgs::PointCloud2ConstPtr & cloud_msg)
{ 
// Container for original & filtered data
  sensor_msgs::PointCloud output;
  sensor_msgs::PointCloud2 output_pc;
  geometry_msgs::Point32 _tempbuffer;
  int numberSampled = (cloud_msg->width)/2;
  cout <<"cloud_msg->width "<<cloud_msg->width<<endl;
  cout <<"numberSampled "<<numberSampled<<endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr PC (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr PC_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*cloud_msg, *PC);

  pcl::RandomSample<pcl::PointXYZ>ransamp;
  ransamp.setInputCloud (PC);
  ransamp.setSample (numberSampled);
  ransamp.setSeed(rand());
  ransamp.filter(*PC_filtered);

  // Convert to ROS data type


  output.header.frame_id="filtered_pc";
  for (size_t i = 0; i < PC_filtered->points.size (); ++i){
	_tempbuffer.x=PC_filtered->points[i].x;
	_tempbuffer.y=PC_filtered->points[i].y;
	_tempbuffer.z=PC_filtered->points[i].z;
	output.points.push_back(_tempbuffer);
}
	
  sensor_msgs::convertPointCloudToPointCloud2(output,output_pc);
   cout <<"cloud_msg->width "<<output_pc.width<<endl;	
  // Publish the data
  _pcPub.publish (output_pc);


}

