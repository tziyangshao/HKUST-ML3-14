#include "PtClListener.h"


using namespace std;


// constructor initializing robot
//PtClListener::PtClListener(ros::NodeHandle _nh): nh(_nh) {
//
//	nh.subsc
//}
PtClListener::PtClListener(int rIDInput, string topicNameInput, ros::NodeHandle nhInput) : robotID(rIDInput), topicName(topicNameInput), nh(nhInput)  {
	subPtCl2 = nh.subscribe<sensor_msgs::PointCloud2>(topicName, 10, &PtClListener::callback_ptCl2Listener, this);
}

PtClListener::~PtClListener(){}


//void PtClListener::callback_ptCl2Listener(const sensor_msgs::PointCloud2ConstPtr& inputPtCl, CentralStorage* storage) {
void PtClListener::callback_ptCl2Listener(const sensor_msgs::PointCloud2ConstPtr& inputPtCl) {
	pcl::fromROSMsg(*inputPtCl,this->localPtCl);
}



