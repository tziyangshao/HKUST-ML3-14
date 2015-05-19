#include "pc_fetcher.h"

using namespace std; 
using namespace sensor_msgs;
using namespace std_msgs;

pc_fetcher::pc_fetcher(){
    _kfSub = _nh.subscribe("/lsd_slam_r1/graph", 10, &pc_fetcher::kfCallback, this);
    _pcSub = _nh.subscribe("/lsd_slam_r1/keyframes", 20, &pc_fetcher::pcCallback, this);
    _pcPub = _nh.advertise<sensor_msgs::PointCloud2>("/slave_robot1/pointcloud2", 5);
    _kFMsgStampedPub = _nh.advertise<slave_robot::keyframeMsgStamped>("/slave_robot1/kFMsgStamped", 5);
    _kFGraphMsgStampedPub = _nh.advertise<slave_robot::keyframeGraphMsgStamped>("/slave_robot1/kFGraphMsgStamped", 5);
}

pc_fetcher::~pc_fetcher(){
	//destructor to release the memory
}

void pc_fetcher::pcCallback(slave_robot::keyframeMsgConstPtr msg){

	this->_cloud2.data.clear();
	this->_pointcloud.points.clear();

    this->readkFMsg(msg);
    this->computeCloud();
    this->fillCloudMsg();

    this->copykFMsgTokFMsgStamped(msg);


    this->_kFGraphMsgStamped.header.stamp = this->_kFMsgStamped.header.stamp;
    this->_pcPub.publish(this->_cloud2);
    this->_kFMsgStampedPub.publish(this->_kFMsgStamped);
	this->_kFGraphMsgStampedPub.publish(this->_kFGraphMsgStamped);
}

void pc_fetcher::kfCallback(slave_robot::keyframeGraphMsgConstPtr graph_msg) {
	kfIndex=(uint32_t)graph_msg->numFrames;

    this->copykFGraphMsgTokFGraphMsgStamped(graph_msg);
}

void pc_fetcher::readkFMsg(slave_robot::keyframeMsgConstPtr msg) {

	ros::Time timeStampTmp(msg->time);
    this->_timeStamp = timeStampTmp;

	this->_fx = msg->fx;
    this->_fy = msg->fy;
    this->_cx = msg->cx;
    this->_cy = msg->cy;

    this->_fxi = 1/this->_fx;
    this->_fyi = 1/this->_fy;
    this->_cxi = -this->_cx / this->_fx;
    this->_cyi = -this->_cy / this->_fy;

    this->_width = msg->width;
    this->_height = msg->height;

    this->_originalInput = new InputPointDense[this->_width*this->_height];
    memcpy(this->_originalInput, msg->pointcloud.data(), this->_width*this->_height*sizeof(InputPointDense));
    memcpy(this->_camToWorld.data(), msg->camToWorld.data(), 7*sizeof(float));

}

void pc_fetcher::computeCloud() {
    geometry_msgs::Point32 tempbuffer;

    for(int y=1;y<this->_height-1;y++) {
		for(int x=1;x<this->_width-1;x++) {

			if(this->_originalInput[x+y*this->_width].idepth <= 0) {
				continue;
			}

			float depth = 1 / this->_originalInput[x+y*this->_width].idepth;
			float depth4 = depth*depth; depth4*= depth4;

			if(this->_originalInput[x+y*this->_width].idepth_var * depth4 > this->scaledDepthVarTH) {
				continue;
			}
			if(this->_originalInput[x+y*this->_width].idepth_var * depth4 * this->_camToWorld.scale()*this->_camToWorld.scale() > this->scaledDepthVarTH) {
				continue;
			}
			if(minNearSupport > 1) {
				int nearSupport = 0;

				for(int dx=-1;dx<2;dx++) {
						for(int dy=-1;dy<2;dy++) {
							int idx = x+dx+(y+dy)*this->_width;
							if(this->_originalInput[idx].idepth > 0) {
								float diff = this->_originalInput[idx].idepth - 1.0f / depth;
								if(diff*diff < 2*this->_originalInput[x+y*this->_width].idepth_var) {
									nearSupport++;
								}
							}
						}
						if(nearSupport < this->minNearSupport) {
							continue;
						}
				}
			}

			Sophus::Vector3f pt = this->_camToWorld * (Sophus::Vector3f((x*this->_fxi + this->_cxi), (y*this->_fyi + this->_cyi), 1.0) * depth);
			tempbuffer.x=pt[0];
			tempbuffer.y=pt[1];
			tempbuffer.z=pt[2];
			this->_pointcloud.points.push_back(tempbuffer);
		}
    }
}


void pc_fetcher::fillCloudMsg() {
    sensor_msgs::convertPointCloudToPointCloud2(this->_pointcloud,this->_cloud2);

    this->_cloud2.header.seq=this->kfIndex;
    this->_cloud2.header.stamp= this->_timeStamp;
    this->_cloud2.header.frame_id="world";
    this->_cloud2.height = 1; // unordered pt cl
    this->_cloud2.width = this->_width;
}

void pc_fetcher::copykFMsgTokFMsgStamped(slave_robot::keyframeMsgConstPtr msg) {
	ros::Time timeStampTmp(msg->time);

	this->_kFMsgStamped.header.seq = this->kfIndex;
	this->_kFMsgStamped.header.stamp = timeStampTmp;
	this->_kFMsgStamped.header.frame_id = "world";

	this->_kFMsgStamped.id = msg->id;
	this->_kFMsgStamped.time = msg->time;
	this->_kFMsgStamped.isKeyframe = msg->isKeyframe;

	this->_kFMsgStamped.camToWorld = msg->camToWorld;

	this->_kFMsgStamped.fx = msg->fx;
	this->_kFMsgStamped.fy = msg->fy;
	this->_kFMsgStamped.cx = msg->cx;
	this->_kFMsgStamped.cy = msg->cy;
	this->_kFMsgStamped.height = msg->height;
	this->_kFMsgStamped.width = msg->width;

	this->_kFMsgStamped.pointcloud = msg->pointcloud;
}

void pc_fetcher::copykFGraphMsgTokFGraphMsgStamped(slave_robot::keyframeGraphMsgConstPtr msg) {
	this->_kFGraphMsgStamped.header.seq = this->kfIndex;
	this->_kFGraphMsgStamped.header.stamp = this->_timeStamp;
	this->_kFGraphMsgStamped.header.frame_id = "world";

	this->_kFGraphMsgStamped.numFrames = msg->numFrames;
	this->_kFGraphMsgStamped.frameData = msg->frameData;
	this->_kFGraphMsgStamped.numConstraints = msg->numConstraints;
	this->_kFGraphMsgStamped.constraintsData = msg->constraintsData;
}
