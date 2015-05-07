/*
//from KeyFrameDisplay.cpp
#define GL_GLEXT_PROTOTYPES 1

//#include "KeyFrameDisplay.h"
#include <beginner_tutorials/keyframeMsg.h>
#include <stdio.h>
//#include "settings.h"
#include <GL/glx.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include "opencv2/opencv.hpp"
#include "ros/package.h"
//from KeyFrameDisplay.h
#pragma once
#undef Success
//#include <Eigen/Core>
//#include "QGLViewer/qglviewer.h"
//#include "beginner_tutorials/keyframeMsg.h"
*/
//#include "src/KeyFrameDisplay.h"
#include "KeyFrameDisplay.h"

#include <sstream>
#include <fstream>
#include <iostream>
//original
#include "ros/ros.h"
//#include "std_msgs/String.h"
//#include "geometry_msgs/PoseStamped.h"
//#include "sophus/sim3.h"

using namespace std; 

int _filecount=0;
char filename[50];
//initialize -----------------------------
int id;
float _time;
bool isKeyframe;

// camToWorld as serialization of sophus sim(3).
//may change with keyframeGraph-updates.
//float[7] camToWorld 


// camera parameter (fx fy cx cy), width, height
// will never change, but required for display.
float fx;
float fy;
float cx;
float cy;
float fxi;
float fyi;
float cxi;
float cyi;
int height;
int width;
float my_scale;
InputPointDense* originalInput;

// camera pose
// may be updated by kf-graph.

Sophus::Sim3f camToWorld;
//Sophus::Vector3f pt;
//Sophus::Vector3f camera_prop;
// data as InputPointDense (float idepth, float idepth_var, uchar color[4]), width x height
// may be empty, in that case no associated pointcloud is ever shown.
//int[] pointcloud
	
//===================================================
void chatterCallback(beginner_tutorials::keyframeMsgConstPtr msg)
{

  //position=data.pose.position
  //quat=data.pose.orientation
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
	id = msg->id;
	_time = msg->time;
	originalInput = new InputPointDense[width*height];
	memcpy(originalInput, msg->pointcloud.data(), width*height*sizeof(InputPointDense));
	memcpy(camToWorld.data(), msg->camToWorld.data(), 7*sizeof(float));
	ROS_INFO("Data Heard\n");

	ofstream myfile;
	sprintf(filename,"/home/turtlebot-3/data/pc%04d",_filecount);
  	myfile.open (filename);
	ROS_INFO(filename);
	//myfile.open ("data/pointcloud","w");
  	//myfile << "Data time:\t"<<_time<<"\n";
	//myfile << "fx:\t"<<fx<<"\t"<<"fy:\t"<<fy<<"\t"<< "cx:\t"<<cx<<"\t"<< "cy:\t"<<cy<<"\n";
	//myfile << "fxi:\t"<<fxi<<"\t"<<"fyi:\t"<<fyi<<"\t"<< "cxi:\t"<<cxi<<"\t"<< "cyi:\t"<<cyi<<"\n";
	//myfile << "width:\t"<<width<<"\t"<<"height:\t"<<height<<"\n";
	//myfile << "camToWorld:\t"<<camToWorld.data[0]<<"\t";
	//myfile << "camToWorld:\t"<<camToWorld.data[0]<<"\t"<<camToWorld.data[1]<<"\t"<<camToWorld[2]<<"\t"<<camToWorld[3]<<"\t"<<camToWorld[4]<<"\t"<<camToWorld[5]<<"\t"<<camToWorld[6]<<"\n";
	for(int y=1;y<height-1;y++){
		for(int x=1;x<width-1;x++){
			float depth = 1/originalInput[x+y*width].idepth;
			float depth4 = depth*depth; depth4*= depth4;
			my_scale=camToWorld.scale();
			if(originalInput[x+y*width].idepth<=0)
				continue;		
				//camera_prop((x*fxi + cxi), (y*fyi + cyi), 1);				
			Sophus::Vector3f pt = camToWorld * (Sophus::Vector3f((x*fxi + cxi), (y*fyi + cyi), 1.0) * depth); 
			myfile <<pt[0] <<" "<< pt[1] <<" "<< pt[2] <<"\n";
				//myfile << x<<"\t"<< y <<"\t"<<depth<<"\n";
			}
		}
 	myfile.close();
  //ROS_INFO("PointPosition: [ %f, %f, %f]\t",msg->position.x,msg->position.y,msg->position.z);
  //ROS_INFO("Quat Orientation: [ %f, %f, %f, %f]\n",msg->quat.x, msg->quat.y, msg->quat.z, msg->quat.w);
	_filecount++;
	ROS_INFO("Chatter calling back to...");
	//ROS_INFO(filename);
}
// %EndTag(CALLBACK)%

int main(int argc, char **argv)
{
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
  ros::init(argc, argv, "listener");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

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
// %Tag(SUBSCRIBER)%
  ros::Subscriber sub = n.subscribe("/lsd_slam/keyframes", 1000, chatterCallback);
// %EndTag(SUBSCRIBER)%

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
// %Tag(SPIN)%
  ros::spin();
// %EndTag(SPIN)%

  return 0;
}
// %EndTag(FULLTEXT)%
