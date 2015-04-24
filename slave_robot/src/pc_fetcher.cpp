#pragma once
#undef Success

#include "KeyFrameDisplay.h"

#include <sstream>
#include <fstream>
#include <iostream>
#include "ros/ros.h"
// -----------------------------
// for image
// -----------------------------
//#include <message_filters/subscriber.h>
//#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

using namespace std; 
using namespace sensor_msgs;


// -----------------------------
// for pointcloud recovery
// -----------------------------
//parameters of the camera
float fx, fy, cx, cy, fxi, fyi, cxi, cyi;
int height, width;
InputPointDense* originalInput;
// camera pose
// may be updated by kf-graph.
Sophus::Sim3f camToWorld;
// -----------------------------
// for keyframe 
// -----------------------------
bool keyframe_trigger=0;
sensor_msgs::Image keyframe;
sensor_msgs::PointCloud pointcloud;
sensor_msgs::PointCloud2 cloud2;

	
void pc_Callback(slave_robot::keyframeMsgConstPtr msg){
	geometry_msgs::Point32 tempbuffer;
	int point_count=0;
	fx = msg->fx;
	fy = msg->fy; 
	cx = msg->cx;
	cy = msg->cy;

	fxi = 1/fx;
	fyi = 1/fy;
	cxi = -cx / fx;
	cyi = -cy / fy;

	width = msg->width;
	height = msg->height;
	//pointcloud.header.stamp = msg->time;
	originalInput = new InputPointDense[width*height];
	memcpy(originalInput, msg->pointcloud.data(), width*height*sizeof(InputPointDense));
	memcpy(camToWorld.data(), msg->camToWorld.data(), 7*sizeof(float));
	// this is the place where they converte uint8 into float, if you
	// dun get it, there is a link, 
        //http://answers.ros.org/question/72469/roscpp-convert-uint8-into-a-known-struct/
	for(int y=1;y<height-1;y++){
		for(int x=1;x<width-1;x++){
			float depth = 1/originalInput[x+y*width].idepth;
			float depth4 = depth*depth; depth4*= depth4;
			if(originalInput[x+y*width].idepth<=0)
				continue;					
			Sophus::Vector3f pt = camToWorld * (Sophus::Vector3f((x*fxi + cxi), (y*fyi + cyi), 1.0) * depth);
			tempbuffer.x=pt[0];
			tempbuffer.y=pt[1];
			tempbuffer.z=pt[2];
			pointcloud.points.push_back(tempbuffer);
			point_count++;
			}
		}
	cout<< point_count <<endl;
	sensor_msgs::convertPointCloudToPointCloud2(pointcloud,cloud2);	
	ROS_INFO("pc calling back...");
}

void keyframecount_Callback(slave_robot::keyframeGraphMsgConstPtr graph_msg){
	pointcloud.header.seq= graph_msg-> numFrames;
	pointcloud.header.frame_id = "robot";
	keyframe_trigger=1;
	ROS_INFO("keyframe calling back...");
}

void image_Callback(const ImageConstPtr& image){
	keyframe = *image;
}

int main(int argc, char **argv){
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "listening_keyframe");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle slave_robot;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber pc_sub = slave_robot.subscribe("/lsd_slam/keyframes", 10, pc_Callback);
  ros::Subscriber keyframecount_sub = slave_robot.subscribe("/lsd_slam/graph", 10, keyframecount_Callback);
  ros::Subscriber image_sub = slave_robot.subscribe("/usb_cam_node/image_raw", 10, image_Callback);
  ros::Publisher image_pub = slave_robot.advertise<sensor_msgs::Image>("/slave_robot/keyFrame", 5);
  ros::Publisher pc_pub = slave_robot.advertise<sensor_msgs::PointCloud2>("/slave_robot/pointcloud", 5);
  while (ros::ok()){
    if(keyframe_trigger==1){
      image_pub.publish(keyframe);
      pc_pub.publish(cloud2);
      keyframe_trigger=0;
    }
    ros::spinOnce();
/*
    if (pointcloud.points.size() >0){
    cout<< pointcloud.points.size() <<endl;
}
*/
    pointcloud.points.clear();
}
  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  return 0;
}
