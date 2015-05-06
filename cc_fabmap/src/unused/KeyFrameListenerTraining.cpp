#include "KeyFrameListenerTraining.h"


using namespace std;
using namespace cv;


// constructor initializing robot
KeyFrameListenerTraining::KeyFrameListenerTraining(int rID, int nbrTrainImgs, string topicName, ros::NodeHandle nh, CentralStorage* storage) :
		m_robotID(rID),
		m_nbrTrainImgs(nbrTrainImgs),
		m_it(nh),
		m_trainImgCtr(0)
{
	m_trainingDataCollection = true;
	m_vocabularyGen = false;

	m_sub = m_it.subscribe(topicName, 10, boost::bind(&KeyFrameListenerTraining::callback_collectTrainImgDescriptors, this, _1, storage) );
}

KeyFrameListenerTraining::~KeyFrameListenerTraining(){}


void KeyFrameListenerTraining::callback_collectTrainImgDescriptors(const sensor_msgs::ImageConstPtr& imgMsg, CentralStorage* storage) {

	int skip = 20; // skip (skip) imgs
	try	{
		m_trainCvPtr = cv_bridge::toCvCopy(imgMsg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	
	// ---- START Collect descriptors of all Images (for generating visual words later on) ---------------------------------------------------------		
	if(m_trainingDataCollection==true && storage->stopDataCollection == false)
	{

		if((m_trainImgCtr % skip) == 0) // pick every (skip) img of video stream
		{
	
			cout << "Detecting keypoints, computing descriptors of training image " << m_trainImgs.size() << endl;

			// Collect descriptors of all Images
			Mat trainDescriptorsTmp; // to save descriptors of current training Image
			vector<KeyPoint> trainKPtsTmp; // detected keypoints to save keypoints of current training Image
	
			(storage->detector)->detect(m_trainCvPtr->image,trainKPtsTmp);

			if(trainKPtsTmp.size()>20) {

				(storage->extractor)->compute(m_trainCvPtr->image,trainKPtsTmp,trainDescriptorsTmp);

				m_trainKPts.push_back(trainKPtsTmp);
				m_trainImgDescr.push_back(trainDescriptorsTmp);
				m_trainImgs.push_back(m_trainCvPtr->image);


				HelperFcts::displayImageKPts("currentTrainingImage", m_trainCvPtr->image, trainKPtsTmp);
				ostringstream convImgName;
				convImgName << "trainImgs/robot" << m_robotID << "TrainImg" << m_trainImgCtr << ".jpg";
				string imgName = convImgName.str();
				HelperFcts::saveImage(m_trainCvPtr->image, imgName);
			}
		}
		m_trainImgCtr = m_trainImgCtr+1;
	}

	if((m_trainImgs.size() >= m_nbrTrainImgs && m_trainingDataCollection == true) || (storage->stopDataCollection == true && m_trainingDataCollection == true))
	{
		m_trainingDataCollection = false;
		m_vocabularyGen = true;
		storage->stopDataCollection = true;
	}
	// ---- END Collect descriptors of all Images (for generating visual words later on) ---------------------------------------------------------
}
