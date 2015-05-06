#ifndef KEYFRAMELISTENERTRAINING_H
#define KEYFRAMELISTENERTRAINING_H

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "CentralStorage.h"


using namespace std;
using namespace cv;



/*! \class KeyFrameListenerTraining
 * \brief Class listening to incoming images, each instance of the class listens to one robot -> do training
 */
class KeyFrameListenerTraining
{
public:

	int m_robotID;

	string m_topicName;
	image_transport::ImageTransport m_it;
	image_transport::Subscriber m_sub;

	bool m_trainingDataCollection;
	bool m_vocabularyGen;
	
	int m_nbrTrainImgs; ///< nbr of images on which prior should be trained for fabmap = 100
	int m_trainImgCtr; ///< counts current nbr of train imgs = 0
	
	keyFrame m_kFrame; ///< stores necessary information about one keyframe
	cv_bridge::CvImagePtr m_trainCvPtr; ///< save ros sensor img to opencv img
	
	Mat m_trainImgDescr; ///< all SURF descrs. of all trainImgs of 1 robot
	vector<vector<KeyPoint> > m_trainKPts; ///< vec<KPts>: all kpts from 1 trainImg of 1 robot, vec<vec<KPt>>: all KPts from all trainImgs from 1 robot
	vector<Mat> m_trainImgs; ///< Mat: trainImg, vec<Mat>: all trainImgs of 1robot
	

	/*! \fn const char RobotListener_Keyframes::RobotListener_Keyframes()
	* \brief Constructor initializing member variables
	*/
	KeyFrameListenerTraining(int rID, int nbrTrainImgs, string topicName, ros::NodeHandle nh, CentralStorage* storage);
	~KeyFrameListenerTraining(); ///< destructor
	

	/*! \fn const char KeyFrameListenerTraining::callback_collectTrainImgDescriptors(const sensor_msgs::ImageConstPtr& msg, CentralStorage* storage)
	* \brief Member function extracting (SURF) descriptors from training Data s.t. vocabulary can be created
	*/
	void callback_collectTrainImgDescriptors(const sensor_msgs::ImageConstPtr& msg, CentralStorage* storage);
	

};


#endif

