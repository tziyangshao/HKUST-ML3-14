#include "PoseListener.h"


using namespace std;


// constructor initializing robot
//PtClListener::PtClListener(ros::NodeHandle _nh): nh(_nh) {
//
//	nh.subsc
//}
PoseListener::PoseListener(int rIDInput, string topicNameInput, ros::NodeHandle nhInput) : robotID(rIDInput), topicName(topicNameInput), nh(nhInput) {
	/// Subscribing to Pose
	subPose = nh.subscribe<geometry_msgs::PoseStamped>(this->topicName, 10, &PoseListener::callback_poseListener, this);
}
PoseListener::~PoseListener(){}


void PoseListener::callback_poseListener(const geometry_msgs::PoseStamped::ConstPtr& inputPose) {
	this->pose = *inputPose;
}



