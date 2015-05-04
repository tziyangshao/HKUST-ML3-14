#include "KeyFrameDisplay.h"

#include <sstream>
#include <fstream>
#include <iostream>
#include "ros/ros.h"
#include <stdint.h>
// -----------------------------
// for image
// -----------------------------
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

using namespace std; 
using namespace sensor_msgs;


class pc_fetcher{
    public:
	static uint32_t kfIndex;
	pc_fetcher();
	~pc_fetcher();	
	static bool keyframeTrigger;
	void pcCallback(slave_robot::keyframeMsgConstPtr msg);
	void kfCallback(slave_robot::keyframeGraphMsgConstPtr graph_msg);
	void imCallback(const ImageConstPtr& image);
	static float scaledDepthVarTH;
	static int minNearSupport;
	
    private:
	//parameters of the camera
	float _fx, _fy, _cx, _cy, _fxi, _fyi, _cxi, _cyi;
	int _height, _width;
	InputPointDense* _originalInput;
	// camera pose
	// may be updated by kf-graph.
	Sophus::Sim3f _camToWorld;
	// -----------------------------
	// for pub sub
	// -----------------------------
	sensor_msgs::Image _keyframe;
	sensor_msgs::PointCloud _pointcloud;
	sensor_msgs::PointCloud2 _cloud2;
	slave_robot::similarityTransformStamped _sim3f;

	ros::NodeHandle _slaveRobot;
	ros::Subscriber _pcSub;
	ros::Subscriber _kfSub;
	ros::Subscriber _imSub;

	ros::Publisher _imPub;
	ros::Publisher _pcPub;
	ros::Publisher _simPub;
};

