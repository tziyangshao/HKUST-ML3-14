#ifndef SYNCLISTENER_H
#define SYNCLISTENER_H

#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>
#include "cc_fabmap/similarityTransformStamped.h"
#include "cc_fabmap/keyframeMsg.h"

#include <cv_bridge/cv_bridge.h>

// PtCl filtering
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

// JUST FOR CHECKING IF SOPHUS EQUAL OWN IMPLEMENTATION
// #include "sophus/sim3.hpp"

#include "CentralStorage.h"


using namespace std;
using namespace std_msgs;
using namespace sensor_msgs;
using namespace geometry_msgs;
using namespace message_filters;

/*! \class SyncListener
 * \brief Class listening to incoming Msgs, each instance of the class listens to one robot
 */
class SyncListener
{
public:

	int m_rID;

	ros::NodeHandle m_nh;

	message_filters::Subscriber<Image> m_subFilImg;
	message_filters::Subscriber<CameraInfo> m_subFilCamInfo;
	message_filters::Subscriber<PointCloud2> m_subFilPtCl;
	message_filters::Subscriber<PoseStamped> m_subFilPose;
	message_filters::Subscriber<cc_fabmap::similarityTransformStamped> m_subSimTrafo;

	typedef sync_policies::ApproximateTime<Image,CameraInfo,PointCloud2,PoseStamped,cc_fabmap::similarityTransformStamped> MySyncPolicy;
	Synchronizer<MySyncPolicy> m_sync;


	// KeyFrame Listener
	keyFrame m_kFrame; ///< stores necessary information about one keyframe
	cv_bridge::CvImagePtr m_testCvPtr; ///< save ros sensor img to opencv img
	vector<KeyPoint> m_testKPts; ///< vec<KPts>: all kpts from 1 testImg of 1 robot
	// Cam Info Listener
	// PtCl Listener
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_localPtCl;
	// Pose Listener
	geometry_msgs::PoseStamped m_localPose;
	// Similarity Trafo Listener
	Eigen::Translation<float,3> m_sim3LocalPoseTransl;
	Eigen::Quaternion<float> m_sim3LocalPoseQuat;
	float m_sim3LocalPoseScale;

//	JUST FOR CHECKING IF SOPHUS EQUAL OWN IMPLEMENTATION
//	Sophus::Sim3f m_sim3LocalPose;









	/*! \fn const char SyncListener::SyncListener()
	* \brief Constructor initializing member variables
	*/
//	SyncListener(ros::NodeHandle _nh);
	SyncListener(int rID, string imgTopicName, string camInfoTopicName, string ptClTopicName, string poseTopicName, string simTrafoTopicName, ros::NodeHandle nh, CentralStorage* storage);
	~SyncListener(); ///< destructor

	/*! \fn const char SyncListener::callback_SyncListener(const sensor_msgs::ImageConstPtr& msg, CentralStorage* storage)
	* \brief Member function listening to incoming PtCls
	*/
	void callback_SyncListener(const ImageConstPtr& kFMsg, const CameraInfoConstPtr& camInfoMsg, const PointCloud2ConstPtr& ptClMsg, const PoseStamped::ConstPtr& poseMsg, const cc_fabmap::similarityTransformStamped::ConstPtr& simTrafoMsg, CentralStorage* storage);
	//void callback_SyncListener(const ImageConstPtr& kFMsg, const CameraInfoConstPtr& camInfoMsg, const PointCloud2ConstPtr& ptClMsg, const PoseStamped::ConstPtr& poseMsg, CentralStorage* storage);

	void voxelFilterPtCl(CentralStorage* storage);

	void clearData();
};


#endif

