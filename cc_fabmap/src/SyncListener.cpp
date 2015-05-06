#include "SyncListener.h"


using namespace std;
using namespace sensor_msgs;
using namespace geometry_msgs;
using namespace message_filters;


// constructor initializing robot
SyncListener::SyncListener(int rID, string imgTopicName, string camInfoTopicName, string ptClTopicName, string poseTopicName, string simTrafoTopicName, ros::NodeHandle nh, CentralStorage* storage) :
		m_rID(rID),
		m_nh(nh),
		m_subFilImg(nh,imgTopicName,100),
		m_subFilCamInfo(nh,camInfoTopicName,100),
		m_subFilPtCl(nh,ptClTopicName,100),
		m_subFilPose(nh,poseTopicName,100),
		m_subSimTrafo(nh,simTrafoTopicName,100),
		m_sync(MySyncPolicy(100), m_subFilImg, m_subFilCamInfo, m_subFilPtCl, m_subFilPose, m_subSimTrafo),
		m_localPtCl(new pcl::PointCloud<pcl::PointXYZ>)
{
	m_sync.registerCallback(boost::bind(&SyncListener::callback_SyncListener, this, _1, _2, _3, _4, _5, storage));
}

SyncListener::~SyncListener(){}


void SyncListener::callback_SyncListener(const ImageConstPtr& kFMsg, const CameraInfoConstPtr& camInfoMsg, const PointCloud2ConstPtr& ptClMsg, const PoseStamped::ConstPtr& poseMsg, const cc_fabmap::similarityTransformStamped::ConstPtr& simTrafoMsg, CentralStorage* storage) {

	// Key Frame Listener
	try	{
		m_testCvPtr = cv_bridge::toCvCopy(kFMsg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	this->m_kFrame.img = m_testCvPtr->image;
	this->m_kFrame.rID = this->m_rID;
	(storage->detector)->detect(this->m_kFrame.img,this->m_kFrame.KPts);

	// Cam Info Listener
	// cam mat stays constant --> no need to process anything
	storage->camModel.fromCameraInfo(camInfoMsg);

	// PtCl Listener
	pcl::fromROSMsg( *ptClMsg, *this->m_localPtCl );
	// Downsample PtCl
	//this->voxelFilterPtCl(storage);

	// Pose Listener
	//this->m_pose = *poseMsg;r

	// Similarity Transform Listener (7DOF)
	// simTrafoMsg contains the coordinate Transformation from Camera Frame of the corresponding robot to the initial robot frame
	// By taking the inverse, the pose of the Frame is revealed
	float simTrafoFloatTmp[7];
	for(int i=0;i<7;++i) {
		simTrafoFloatTmp[i] = simTrafoMsg->simTrafo[i];
	}

	HelperFcts::trafoFloat7ToQuatTranslScale(simTrafoFloatTmp, this->m_sim3LocalPoseQuat, this->m_sim3LocalPoseTransl, this->m_sim3LocalPoseScale);
	HelperFcts::getInvQuatTranslScale(this->m_sim3LocalPoseQuat, this->m_sim3LocalPoseQuat, this->m_sim3LocalPoseTransl, this->m_sim3LocalPoseTransl, this->m_sim3LocalPoseScale, this->m_sim3LocalPoseScale);
	Eigen::Matrix4f simTrafoTmp;
	simTrafoTmp.setIdentity(4,4);
	HelperFcts::trafoFloat7Matrix4f(simTrafoFloatTmp, simTrafoTmp);
	HelperFcts::eigenQuatTranslPoseToROSPose(this->m_sim3LocalPoseQuat, this->m_sim3LocalPoseTransl, this->m_localPose);


	// JUST FOR CHECKING IF SOPHUS EQUAL OWN IMPLEMENTATION
//	memcpy(m_sim3LocalPose.data(), simTrafoMsg->sim3f.data(), 7*sizeof(float));
//	cout << endl << endl<< endl;
//	cout << "Sophus: " << this->m_sim3LocalPose.translation() << endl;
//	cout << this->m_sim3LocalPose.quaternion().w() << " " << this->m_sim3LocalPose.quaternion().x() << " " << this->m_sim3LocalPose.quaternion().y() << " " << this->m_sim3LocalPose.quaternion().z() << endl;
//	cout << "scale: " << this->m_sim3LocalPose.scale() << endl;
//	cout << this->m_sim3LocalPose.quaternion().w()/this->m_sim3LocalPose.scale() << " " << this->m_sim3LocalPose.quaternion().x()/this->m_sim3LocalPose.scale() << " " << this->m_sim3LocalPose.quaternion().y() << " " << this->m_sim3LocalPose.quaternion().z()/this->m_sim3LocalPose.scale() << endl << endl;
//	cout << "Eigen: " << this->m_localPose.pose.position.x << " " << this->m_localPose.pose.position.y << " " << this->m_localPose.pose.position.z << endl;
//	cout << this->m_localPose.pose.orientation.w << " " << this->m_localPose.pose.orientation.x << " " << this->m_localPose.pose.orientation.y << " " << this->m_localPose.pose.orientation.z << endl;
//	cout << "scale: " << this->m_sim3LocalPoseScale << endl;
//	cout << endl << endl<< endl;

}



void SyncListener::voxelFilterPtCl(CentralStorage* storage) {

	// Create the filtering object
	pcl::VoxelGrid< pcl::PointXYZ > sor;

	sor.setInputCloud ( (this->m_localPtCl) );
	sor.setLeafSize (0.001f, 0.001f, 0.001f);
	sor.filter ( *(this->m_localPtCl) );

}

void SyncListener::clearData() {

	this->m_kFrame.img.release();
	this->m_kFrame.KPts.clear();
	this->m_kFrame.bowDescriptor.release();
	this->m_kFrame.fID = 0;
	this->m_kFrame.rID = 0;

	this->m_testKPts.clear();
	this->m_localPtCl->clear();

}

