#include "pc_fetcher.h"

using namespace std; 
using namespace sensor_msgs;
using namespace std_msgs;

pc_fetcher::pc_fetcher(){
    _imSub = _slaveRobot.subscribe("/usb_cam_r1/image_rect", 30, &pc_fetcher::imCallback, this);
    _kfSub = _slaveRobot.subscribe("/lsd_slam_r2/graph", 10, &pc_fetcher::kfCallback, this);
    _pcSub = _slaveRobot.subscribe("/lsd_slam_r2/keyframes", 20, &pc_fetcher::pcCallback, this);
    _imPub = _slaveRobot.advertise<sensor_msgs::Image>("/slave_robot1/keyFrame", 5);
    _pcPub = _slaveRobot.advertise<sensor_msgs::PointCloud2>("/slave_robot1/pointcloud2", 5);
    _simPub = _slaveRobot.advertise<slave_robot::similarityTransformStamped>("/slave_robot1/similarityTransformation", 5);
/*
    _imSub = _slaveRobot.subscribe("/usb_cam_r2/image_raw", 30, &pc_fetcher::imCallback, this);
    _kfSub = _slaveRobot.subscribe("/lsd_slam_r2/graph", 10, &pc_fetcher::kfCallback, this);
    _pcSub = _slaveRobot.subscribe("/lsd_slam_r2/keyframes", 20, &pc_fetcher::pcCallback, this);
    _imPub = _slaveRobot.advertise<sensor_msgs::Image>("/slave_robot2/keyFrame", 5);
    _pcPub = _slaveRobot.advertise<sensor_msgs::PointCloud2>("/slave_robot2/pointcloud2", 5);
    _simPub = _slaveRobot.advertise<slave_robot::similarityTransformStamped>("/slave_robot2/similarityTransformation", 5);
*/
}
pc_fetcher::~pc_fetcher(){
	//destructor to release the memory
}

void pc_fetcher::pcCallback(slave_robot::keyframeMsgConstPtr msg){
    _cloud2.data.clear();
    _pointcloud.points.clear();
    geometry_msgs::Point32 _tempbuffer;
    ros::Time timeStamp(msg->time);
    _fx = msg->fx;
    _fy = msg->fy; 
    _cx = msg->cx;
    _cy = msg->cy;

    _fxi = 1/_fx;
    _fyi = 1/_fy;
    _cxi = -_cx / _fx;
    _cyi = -_cy / _fy;
  
    _width = msg->width;
    _height = msg->height;
    //pointcloud.header.stamp = msg->time;
    _originalInput = new InputPointDense[_width*_height];
    memcpy(_originalInput, msg->pointcloud.data(), _width*_height*sizeof(InputPointDense));
    memcpy(_camToWorld.data(), msg->camToWorld.data(), 7*sizeof(float));

    for(int y=1;y<_height-1;y++)
	for(int x=1;x<_width-1;x++){
	    if(_originalInput[x+y*_width].idepth <= 0) continue;
		float depth = 1 / _originalInput[x+y*_width].idepth;
		float depth4 = depth*depth; depth4*= depth4;
		if(_originalInput[x+y*_width].idepth_var * depth4 > scaledDepthVarTH) continue;
		if(_originalInput[x+y*_width].idepth_var * depth4 * _camToWorld.scale()*_camToWorld.scale() > scaledDepthVarTH) continue;
		if(minNearSupport > 1){
		    int nearSupport = 0;
			for(int dx=-1;dx<2;dx++)
    			    for(int dy=-1;dy<2;dy++){
				int idx = x+dx+(y+dy)*_width;
				if(_originalInput[idx].idepth > 0){
				    float diff = _originalInput[idx].idepth - 1.0f / depth;
				    if(diff*diff < 2*_originalInput[x+y*_width].idepth_var)
					nearSupport++;
				}
			    }
		    if(nearSupport < minNearSupport)continue;
		}
	    //Sophus::Vector3f pt = _camToWorld * (Sophus::Vector3f((x*_fxi + _cxi), (y*_fyi + _cyi), 1.0) * depth);
	    Sophus::Vector3f pt = (Sophus::Vector3f((x*_fxi + _cxi), (y*_fyi + _cyi), 1.0) * depth);
	    _tempbuffer.x=pt[0];
	    _tempbuffer.y=pt[1];
            _tempbuffer.z=pt[2];
	    _pointcloud.points.push_back(_tempbuffer);
	}

    sensor_msgs::convertPointCloudToPointCloud2(_pointcloud,_cloud2);	
    ROS_INFO("pc calling back...");
    _cloud2.header.seq=kfIndex;
    _cloud2.header.stamp= timeStamp;
    _cloud2.header.frame_id="world";
    cout << _cloud2.header.seq << endl;
    _keyframe.header.seq=kfIndex;
    cout << _keyframe.header.seq << endl;

// Filling in Similarity Trafo Info
	_sim3f.header.stamp = timeStamp;
	_sim3f.sim3f = msg->camToWorld;
	_simPub.publish(_sim3f);
    _imPub.publish(_keyframe);
    _pcPub.publish(_cloud2);
}

void pc_fetcher::kfCallback(slave_robot::keyframeGraphMsgConstPtr graph_msg){
	kfIndex=(uint32_t)graph_msg-> numFrames;
}
void pc_fetcher::imCallback(const ImageConstPtr& image){
	_keyframe = *image;
}
