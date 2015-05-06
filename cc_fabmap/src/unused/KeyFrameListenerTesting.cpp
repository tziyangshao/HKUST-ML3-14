#include "KeyFrameListenerTesting.h"


using namespace std;
using namespace cv;


// constructor initializing robot
KeyFrameListenerTesting::KeyFrameListenerTesting(int rID, string topicName, ros::NodeHandle nh, CentralStorage* storage) :
		m_robotID(rID),
		m_it(nh)
{
	m_sub = m_it.subscribe(topicName, 10, boost::bind(&KeyFrameListenerTesting::callback_collectTestKPts, this, _1, storage) );
}

KeyFrameListenerTesting::~KeyFrameListenerTesting(){}



void KeyFrameListenerTesting::callback_collectTestKPts(const sensor_msgs::ImageConstPtr& imgMsg, CentralStorage* storage) {

	try	{
		m_testCvPtr = cv_bridge::toCvCopy(imgMsg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}


	this->m_kFrame.img = m_testCvPtr->image;
	this->m_kFrame.rID = this->m_robotID;

	(storage->detector)->detect(this->m_kFrame.img,this->m_kFrame.KPts);
}


