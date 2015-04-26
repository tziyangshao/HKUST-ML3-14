#include "class.h"

using namespace std; 
using namespace sensor_msgs;

pc_fetcher::pc_fetcher(void){
    _pcSub = _slaveRobot.subscribe("/lsd_slam/keyframes", 20, this->_pcCallback);
    _kfSub = _slaveRobot.subscribe("/lsd_slam/graph", 10, this->_kfCallback);
    _imSub = _slaveRobot.subscribe("/usb_cam_node/image_raw", 30, pc_fetcher::_imCallback);
    _imPub = _slaveRobot.advertise<sensor_msgs::Image>("/slave_robot/keyFrame", 5);
    _pcPub = _slaveRobot.advertise<sensor_msgs::PointCloud2>("/slave_robot/pointcloud", 5);
    while (ros::ok()){
    	if(_keyframeTrigger==1){
	      _imPub.publish(_keyframe);
	      _pcPub.publish(_cloud2);
	      _keyframeTrigger=0;
    	}
    }
}
pc_fetcher::~pc_fetcher(void){
	//destructor to release the memory
}


void pc_fetcher::_pcCallback(slave_robot::keyframeMsgConstPtr msg){
	geometry_msgs::Point32 _tempbuffer;
	_fx = msg->fx;
	_fy = msg->fy; 
	_cx = msg->cx;
	_cy = msg->cy;

	_fxi = 1/fx;
	_fyi = 1/fy;
	_cxi = -cx / fx;
	_cyi = -cy / fy;

	_width = msg->width;
	_height = msg->height;
	//pointcloud.header.stamp = msg->time;
	_originalInput = new InputPointDense[width*height];
	memcpy(_originalInput, msg->pointcloud.data(), width*height*sizeof(InputPointDense));
	memcpy(_camToWorld.data(), msg->camToWorld.data(), 7*sizeof(float));
	// this is the place where they converte uint8 into float, if you
	// dun get it, there is a link, 
        //http://answers.ros.org/question/72469/roscpp-convert-uint8-into-a-known-struct/
	for(int y=1;y<_height-1;y++){
		for(int x=1;x<_width-1;x++){
			float depth = 1/_originalInput[x+y*_width].idepth;
			float depth4 = depth*depth; depth4*= depth4;
			if(_originalInput[x+y*width].idepth<=0)
				continue;					
			Sophus::Vector3f pt = _camToWorld * (Sophus::Vector3f((x*fxi + cxi), (y*fyi + cyi), 1.0) * depth);
			_tempbuffer.x=pt[0];
			_tempbuffer.y=pt[1];
			_tempbuffer.z=pt[2];
			_pointcloud.points.push_back(_tempbuffer);
			}
		}
	sensor_msgs::convertPointCloudToPointCloud2(_pointcloud,_cloud2);	
	ROS_INFO("pc calling back...");
}

void pc_fetcher::_kfCallback(slave_robot::keyframeGraphMsgConstPtr graph_msg){
	_pointcloud.header.seq= graph_msg-> numFrames;
	_keyframeTrigger=1;
}

void pc_fetcher::_imCallback(const ImageConstPtr& image){
	_keyframe = *image;
}

