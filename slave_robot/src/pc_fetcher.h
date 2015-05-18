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
		static bool keyframeTrigger;
		static uint32_t kfIndex;
		static float scaledDepthVarTH;
		static int minNearSupport;

		pc_fetcher();
		~pc_fetcher();
		void pcCallback(slave_robot::keyframeMsgConstPtr msg);
		void kfCallback(slave_robot::keyframeGraphMsgConstPtr graph_msg);
	
    private:
		// time stamp
		ros::Time _timeStamp;

		//parameters of the camera
		float _fx, _fy, _cx, _cy, _fxi, _fyi, _cxi, _cyi;
		int _height, _width;
		InputPointDense* _originalInput;

		// camera pose
		// may be updated by kf-graph.
		Sophus::Sim3f _camToWorld;

		// -----------------------------
		// for subscriber and publisher
		// -----------------------------
		sensor_msgs::PointCloud _pointcloud;
		sensor_msgs::PointCloud2 _cloud2;
		slave_robot::keyframeMsgStamped _kFMsgStamped;
		slave_robot::keyframeGraphMsgStamped _kFGraphMsgStamped;

		ros::NodeHandle _nh;
		ros::Subscriber _pcSub;
		ros::Subscriber _kfSub;

		ros::Publisher _pcPub;
		ros::Publisher _kFMsgStampedPub;
		ros::Publisher _kFGraphMsgStampedPub;

		void computeCloud();
		void readkFMsg(slave_robot::keyframeMsgConstPtr msg);
		void fillCloudMsg();
		void copykFMsgTokFMsgStamped(slave_robot::keyframeMsgConstPtr msg);
		void copykFGraphMsgTokFGraphMsgStamped(slave_robot::keyframeGraphMsgConstPtr msg);

};

