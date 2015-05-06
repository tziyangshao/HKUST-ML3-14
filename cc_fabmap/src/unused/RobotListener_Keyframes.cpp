#include "RobotListener_Keyframes.h"


using namespace std;
using namespace cv;


// constructor initializing robot
RobotListener_Keyframes::RobotListener_Keyframes()
{
	fabmap_dataCollection = true;
	fabmap_vocGen = false;
	
	nbr_trainImgs = 1500;//1000;
	train_ctr = 0;
}

RobotListener_Keyframes::~RobotListener_Keyframes(){}


void RobotListener_Keyframes::callback_collectTrainDescriptors(const sensor_msgs::ImageConstPtr& msg, CentralStorage* storage) {

	int skip = 20; // skip (skip) imgs
	try	{
		train_cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	
	// ---- START Collect descriptors of all Images (for generating visual words later on) ---------------------------------------------------------		
	if(fabmap_dataCollection==true && storage->stopDataCollection == false) 
	{

		if((train_ctr % skip) == 0) // pick every (skip) img of video stream
		{
	
			cout << "Detecting keypoints, computing descriptors of training image " << trainImgs.size() << endl;

			// Collect descriptors of all Images
			Mat trainDescriptors_tmp; // to save descriptors of current training Image
			vector<KeyPoint> train_kpts_tmp; // detected keypoints to save keypoints of current training Image
	
			(storage->detector)->detect(train_cv_ptr->image,train_kpts_tmp);

			if(train_kpts_tmp.size()>20) {

				(storage->extractor)->compute(train_cv_ptr->image,train_kpts_tmp,trainDescriptors_tmp);

				trainKPts.push_back(train_kpts_tmp);
				trainDescriptors.push_back(trainDescriptors_tmp);
				trainImgs.push_back(train_cv_ptr->image);


				HelperFcts::displayImageKPts("currentTrainingImage", train_cv_ptr->image, train_kpts_tmp);
				ostringstream convImgName;
				convImgName << "trainImgs/robot" << this->robotID << "TrainImg" << train_ctr << ".jpg";
				string imgName = convImgName.str();
				HelperFcts::saveImage(train_cv_ptr->image, imgName);
			}
		}
		train_ctr = train_ctr+1;
	}

	if((trainImgs.size() >= nbr_trainImgs && fabmap_dataCollection == true) || (storage->stopDataCollection == true && fabmap_dataCollection == true))
	{
		fabmap_dataCollection = false;
		fabmap_vocGen = true;
		storage->stopDataCollection = true;
	}
	// ---- END Collect descriptors of all Images (for generating visual words later on) ---------------------------------------------------------
}


void RobotListener_Keyframes::callback_collectTestKPts(const sensor_msgs::ImageConstPtr& msg, CentralStorage* storage) {


	try	{
		test_cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}


	this->kFrame.img = test_cv_ptr->image;
	this->kFrame.rID = this->robotID;

	vector<KeyPoint> test_kpts_tmp; // detected keypoints of current test image
	(storage->detector)->detect(this->kFrame.img,this->kFrame.KPts);

}


