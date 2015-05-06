#ifndef KEYFRAMELISTENERTESTING_H
#define KEYFRAMELISTENERTESTING_H



#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <opencv2/opencv.hpp>
#include "opencv2/nonfree/nonfree.hpp"


#include "CentralStorage.h"


using namespace std;
using namespace cv;



/*! \class KeyFrameListenerTesting
 * \brief Class listening to incoming images, each instance of the class listens to one robot
 */
class KeyFrameListenerTesting
{
public:

	int m_robotID;

	string m_topicName;
	image_transport::ImageTransport m_it;
	image_transport::Subscriber m_sub;

	keyFrame m_kFrame; ///< stores necessary information about one keyframe
	cv_bridge::CvImagePtr m_testCvPtr; ///< save ros sensor img to opencv img
	vector<KeyPoint> m_testKPts; ///< vec<KPts>: all kpts from 1 testImg of 1 robot


	/*! \fn const char KeyFrameListenerTesting::KeyFrameListenerTesting()
	* \brief Constructor initializing member variables
	*/
	KeyFrameListenerTesting(int rID, string topicName, ros::NodeHandle nh, CentralStorage* storage);
	~KeyFrameListenerTesting(); ///< destructor
	
	



	/*! \fn const char KeyFrameListenerTesting::callback_collectTestKPts(const sensor_msgs::ImageConstPtr& msg, CentralStorage* storage)
	* \brief Member function extracting (SURF) descriptors from testing Data s.t. can run FABMAP for place recognition
	*/
	void callback_collectTestKPts(const sensor_msgs::ImageConstPtr& msg, CentralStorage* storage);
	
};


#endif

