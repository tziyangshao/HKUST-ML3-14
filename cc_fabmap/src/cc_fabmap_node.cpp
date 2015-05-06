#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/Image.h>
#include <message_filters/time_synchronizer.h>
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/nonfree/nonfree.hpp"

#include "KeyFrameListenerTraining.h"
#include "SyncListener.h"


using namespace std;
using namespace cv;
using namespace opengv;


void helperStoreTrainData(CentralStorage* storage, KeyFrameListenerTraining & listenerR1, KeyFrameListenerTraining & listenerR2) {

	storage->trainDescriptors_allRobots.push_back(listenerR1.m_trainImgDescr);
	storage->trainDescriptors_allRobots.push_back(listenerR2.m_trainImgDescr);

	std::vector<cv::Mat>::iterator itImg1;
	for(itImg1 = listenerR1.m_trainImgs.begin(); itImg1 != listenerR1.m_trainImgs.end(); ++itImg1) {
		storage->trainImgs_allRobots.push_back(*itImg1);
	}
	std::vector<cv::Mat>::iterator itImg2;
	for(itImg2 = listenerR2.m_trainImgs.begin(); itImg2 != listenerR2.m_trainImgs.end(); ++itImg2) {
		storage->trainImgs_allRobots.push_back(*itImg2);
	}


	std::vector<vector<KeyPoint> >::iterator itKPts1;
	for(itKPts1 = listenerR1.m_trainKPts.begin(); itKPts1 != listenerR1.m_trainKPts.end(); ++itKPts1) {
		storage->trainKPts_allRobots.push_back(*itKPts1);
	}
	std::vector<vector<KeyPoint> >::iterator itKPts2;
	for(itKPts2 = listenerR2.m_trainKPts.begin(); itKPts2 != listenerR2.m_trainKPts.end(); ++itKPts2) {
		storage->trainKPts_allRobots.push_back(*itKPts2);
	}
}

void helperSaveTrainData(CentralStorage* storage) {

	FileStorage fs1("vocabulary.yml", FileStorage::WRITE);
	fs1 << "vocabulary" << storage->vocabulary;
	fs1.release();

	FileStorage fs2("trainBOWDescriptors_allRobots.yml", FileStorage::WRITE);
	fs2 << "trainBOWDescriptors_allRobots" << storage->trainBOWDescriptors_allRobots;
	fs2.release();

	FileStorage fs3("clTree.yml", FileStorage::WRITE);
	fs3 << "clTree" << storage->clTree;
	fs3.release();

}

void helperReadTrainData(CentralStorage* storage) {

/*
	// load training Images
//	fs.open(string("trainImgs_allRobots.yml"), FileStorage::READ);
//	fs["trainImgs_allRobots"] >> storage->trainImgs_allRobots;
//	if (storage->trainImgs_allRobots.empty()) {
//		cerr << "trainImgs_allRobots not found" << endl;
//		return -1;
//	}
//	fs.release();

	// load training KeyPoints
//	fs.open(string("trainKPts_allRobots.yml"), FileStorage::READ);
//	fs["trainKPts_allRobots"] >> storage->trainKPts_allRobots;
//	if (storage->trainKPts_allRobots.empty()) {
//		cerr << "trainKPts_allRobots not found" << endl;
//		return -1;
//	}
//	fs.release();

	// load SURF descriptors of training data
//	fs.open(string("trainDescriptors_allRobots.yml"), FileStorage::READ);
//	fs["trainDescriptors_allRobots"] >> storage->trainDescriptors_allRobots;
//	if (storage->trainDescriptors_allRobots.empty()) {
//		cerr << "Training Data SURF Descriptors not found" << endl;
//		return -1;
//	}
//	fs.release();
*/

	FileStorage fs;

	// load vocabulary
	fs.open(string("vocabulary.yml"), FileStorage::READ);
	fs["vocabulary"] >> storage->vocabulary;
	if (storage->vocabulary.empty()) {
		cerr << "Vocabulary not found" << endl;
//		return -1;
	}
	fs.release();

	// load trainBOWDescriptors_allRobots
	fs.open(string("trainBOWDescriptors_allRobots.yml"), FileStorage::READ);
	fs["trainBOWDescriptors_allRobots"] >> storage->trainBOWDescriptors_allRobots;
	if (storage->trainBOWDescriptors_allRobots.empty()) {
		cerr << "trainBOWDescriptors_allRobots not found" << endl;
//		return -1;
	}
	fs.release();

	// load clTree
	fs.open(string("clTree.yml"), FileStorage::READ);
	fs["clTree"] >> storage->clTree;
	if (storage->clTree.empty()) {
		cerr << "clTree not found" << endl;
//		return -1;
	}
	fs.release();
}

void helperGetTestData(CentralStorage* storage) {

	//std::vector<keyFrame>::iterator itKF;
	std::map<int,keyFrame>::iterator itKF;
	for(itKF = storage->kFrames.begin(); itKF != storage->kFrames.end(); ++itKF) {
		storage->testBOWDescriptors_allRobots.push_back((*itKF).second.bowDescriptor);
	}

}

void saveTestImgs(CentralStorage* storage, SyncListener & listener_r1, SyncListener & listener_r2) {
	// NO IMPLEMENTATION YET
}

void helperDispOutput(const CentralStorage& storage, bool saveMatchMatrix=false, string matchMatrixNameImg="", string matchMatrixNameFile="") {

	int nbrImgs = storage.testBOWDescriptors_allRobots.size().height; // query img + test imgs
	int queryCtr = 0;

	Mat match_small_f = Mat::zeros(nbrImgs, nbrImgs, CV_32FC1);
	Mat matchMatrixRGB = Mat::zeros(nbrImgs,nbrImgs,CV_32FC3);

	vector<of2::IMatch>::const_iterator l;
	for(l = storage.matches.begin(); l != storage.matches.end(); l++) {
// USEFUL FOR DEBUG:		cout << "queryIdx: " << l->queryIdx << "   imgIdx: " << l->imgIdx << "   match: " << l->match << endl;
		if(l->imgIdx < 0) {
			match_small_f.at<float>(queryCtr, queryCtr) = (l->match*1.0);
			if(storage.kFrames.at(queryCtr).rID == 1) {
				matchMatrixRGB.at<Vec3f>(queryCtr, queryCtr).val[0] = l->match*255.0; // .at(col,row)
			}
			else if(storage.kFrames.at(queryCtr).rID == 2) {
				matchMatrixRGB.at<Vec3f>(queryCtr, queryCtr).val[1] = l->match*255.0;
			}
			queryCtr = queryCtr + 1;

		} else {
			match_small_f.at<float>(queryCtr-1, l->imgIdx) = (l->match*1.0);
			if(storage.kFrames.at(l->imgIdx).rID == 1) {
				matchMatrixRGB.at<Vec3f>(queryCtr-1, l->imgIdx).val[0] = l->match*255.0;
			}
			else if(storage.kFrames.at(l->imgIdx).rID == 2) {
				matchMatrixRGB.at<Vec3f>(queryCtr-1, l->imgIdx).val[1] = l->match*255.0;
			}
		}
	}

	//cout << match_small_f << endl;
	//Mat match_large_f(10*nbrImgs, 10*nbrImgs, CV_32FC1);
	Mat matchMatrixRGB_large = Mat::zeros(10*nbrImgs,10*nbrImgs,CV_32FC3);

	//resize(match_small_f, match_large_f, Size(500, 500), 0, 0, CV_INTER_NN);
	resize(matchMatrixRGB, matchMatrixRGB_large, Size(500, 500), 0, 0, CV_INTER_NN);

	if(saveMatchMatrix) {
		HelperFcts::saveImage(matchMatrixRGB_large,matchMatrixNameImg);
		HelperFcts::saveMatrix(matchMatrixRGB,matchMatrixNameFile);
	}

	/*
	 * 		I0 I1 I2 I3 ... I23 Q
	 * I0
	 * I1
	 * I2
	 * ...
	 *
	 * I23
	 * Q
	 */
	imshow("Confusion match probab Matrix", matchMatrixRGB_large/255.0);

	waitKey(5);
}

void helperTrafoToROSMsg(transformation_t trafo, tf::Transform & trafoROSMsg) {
	tf::Vector3 origin(trafo(0,3),trafo(1,3),trafo(2,3));

	tf::Matrix3x3 rotation(trafo(0,0),trafo(0,1),trafo(0,2),trafo(1,0),trafo(1,1),trafo(1,2),trafo(2,0),trafo(2,1),trafo(2,2));

	tf::Quaternion q;

	trafoROSMsg.setOrigin(origin);
	rotation.getRotation(q);
	q.normalize();
	trafoROSMsg.setRotation(q);
}

void helperPoseToROSTrafoMsg(geometry_msgs::Pose trafo,tf::Transform & trafoROSMsg) {
	tf::Vector3 origin(trafo.position.x, trafo.position.y, trafo.position.z);
	tf::Quaternion q;
	q.setW(trafo.orientation.w);
	q.setX(trafo.orientation.x);
	q.setY(trafo.orientation.y);
	q.setZ(trafo.orientation.z);
	q.normalize();

	trafoROSMsg.setOrigin(origin);
	trafoROSMsg.setRotation(q);
}

void helperMatrix4fToROSTrafoMsg(Eigen::Matrix4f trafo, tf::Transform & trafoROSMsg) {
	tf::Vector3 origin( trafo(0,3), trafo(1,3), trafo(2,3) );

	Eigen::Matrix3f rot = trafo.block(0,0,3,3);
	Eigen::Quaternion<float> qEigen(rot);
	tf::Quaternion q( qEigen.w(), qEigen.x(), qEigen.y(), qEigen.z() );
	q.normalize();

	trafoROSMsg.setOrigin(origin);
	trafoROSMsg.setRotation(q);
}















int main(int argc, char **argv) {

	/// ---- INITIALIZE ---------------------------------------------------------
	bool doTraining = false;
	// can true
	bool doTesting = true;
	// can false	
	bool boolSaveTestImgs = false;
	bool boolNewKF = false;
	bool boolEndAfterFound1Match = false;

	int nbrRobots = 2;

	ros::init(argc, argv, "cc_fabmap");
	ros::NodeHandle nh;

	CentralStorage* storage = new CentralStorage(nbrRobots);



	/// ----------------------------------------
	/// ----------------------------------------
	///
	/// TRAINING -------------------------------
	///
	/// ----------------------------------------
	/// ----------------------------------------
	KeyFrameListenerTraining kFListenerTrainingR1(1, 1500, "/usb_cam_training/image_rect", nh, storage);
	KeyFrameListenerTraining kFListenerTrainingR2(2, 1500, "/usb_cam_training_robot2/image_raw", nh, storage);



	if(doTraining) {
		// spinning
		cout << endl << "TRAINING --------------------------------" << endl;

		initModule_nonfree();

		ros::Rate rTrain(20);
		while(storage->stopDataCollection==false && ros::ok())
		{
			ros::spinOnce();
			rTrain.sleep();
		}


		/// ---- Generate vocabulary (visual words) ---------------------------------------------------------
		if(storage->stopDataCollection==true)
		{
			cout << endl << "store all Training Data in instance of CentralStorage" << endl;
			helperStoreTrainData(storage, kFListenerTrainingR1, kFListenerTrainingR2);

			cout << "Generate Vocabulary" << endl;
			storage->generateVocabulary();
	
			cout << "Set Vocabulary for BOW Descriptor extractor" << endl;
			(storage->bide)->setVocabulary(storage->vocabulary);

			cout << "Compute BOW Descriptors for Training Images" << endl;
			storage->computeTrainBOWDescriptors();

			cout << "Compute Chow Liu Tree based on Training Data" << endl;
			storage->generateCLTree();

			cout << "save all necessary Training Data on harddisk" << endl;
			helperSaveTrainData(storage);

			cout << "Initialize FabMap instance" << endl << endl;
			storage->fabmap = new of2::FabMap2(storage->clTree, 0.39, 0, of2::FabMap::SAMPLED | of2::FabMap::CHOW_LIU);
			storage->fabmap->addTraining(storage->trainBOWDescriptors_allRobots);
		}
	}
	else {
		storage->stopDataCollection = true;

		helperReadTrainData(storage);

		cout << "Set Vocabulary for BOW Descriptor extractor" << endl;
		(storage->bide)->setVocabulary(storage->vocabulary);

		cout << "Initialize FabMap instance" << endl << endl;
		storage->fabmap = new of2::FabMap2(storage->clTree, 0.4, 0, of2::FabMap::SAMPLED | of2::FabMap::CHOW_LIU);
		storage->fabmap->addTraining(storage->trainBOWDescriptors_allRobots);

	}
	cout << "TRAINING OR LOADING finished --------------------------------" << endl << endl;


	

	/// ----------------------------------------
	/// ----------------------------------------
	///
	/// TESTING --------------------------------
	///
	/// ----------------------------------------
	/// ----------------------------------------
	cout << "Initialize synchronized ROS topic listener" << endl;
	SyncListener syncListenerR1(1, "/slave_robot1/keyFrame", "/usb_cam_r1/camera_info", "/slave_robot1/pointcloud2Filtered", "/lsd_slam_r1/pose", "/slave_robot1/similarityTransformation", nh, storage);
	SyncListener syncListenerR2(2, "/slave_robot2/keyFrame", "/usb_cam_r2/camera_info", "/slave_robot2/pointcloud2Filtered", "/lsd_slam_r2/pose", "/slave_robot2/similarityTransformation", nh, storage);

	// ONLY FOR CHECKING IN RVIZ (RUN RVIZ IN GDB MODE: gdb /opt/ros/hydro/lib/rviz/rviz)
	ros::Publisher pub1 = nh.advertise< pcl::PointCloud<pcl::PointXYZ> > ("ptClCheck1", 1);
	ros::Publisher pub2 = nh.advertise< pcl::PointCloud<pcl::PointXYZ> > ("ptClCheck2", 1);
	ros::Publisher pub3 = nh.advertise< pcl::PointCloud<pcl::PointXYZ> > ("ptClCheck3", 1);

	static tf::TransformBroadcaster br;
	tf::Transform trafoROSMsgR1,trafoROSMsgR2,
	trafoROSMsgR1C1, trafoROSMsgC1R1,
	trafoROSMsgR2C2, trafoROSMsgC2R2,
	trafoROSMsgC2C1, trafoROSMsgC1C2;
	helperTrafoToROSMsg(transformation_t::Identity(3,4),trafoROSMsgR1);
	helperTrafoToROSMsg(transformation_t::Identity(3,4),trafoROSMsgR2);
	helperTrafoToROSMsg(transformation_t::Identity(3,4),trafoROSMsgR1C1);
	helperTrafoToROSMsg(transformation_t::Identity(3,4),trafoROSMsgC1R1);
	helperTrafoToROSMsg(transformation_t::Identity(3,4),trafoROSMsgR2C2);
	helperTrafoToROSMsg(transformation_t::Identity(3,4),trafoROSMsgC2R2);
	helperTrafoToROSMsg(transformation_t::Identity(3,4),trafoROSMsgC2C1);
	helperTrafoToROSMsg(transformation_t::Identity(3,4),trafoROSMsgC1C2);

	// --------------------------------------


	if(doTesting) {
		cout << endl << "TESTING --------------------------------" << endl << endl;
		ros::Rate rTest(3);

		while(ros::ok() && storage->testImgCtr < 2000)
		{
			ros::spinOnce();

			ostringstream convImgNameR1, convMatchMatrixNameImgR1, convMatchMatrixNameFileR1;
			ostringstream convImgNameR2, convMatchMatrixNameImgR2, convMatchMatrixNameFileR2;
			string imgNameR1, matchMatrixNameImgR1, matchMatrixNameFileR1;
			string imgNameR2, matchMatrixNameImgR2, matchMatrixNameFileR2;

			if(!syncListenerR1.m_kFrame.img.empty() && syncListenerR1.m_kFrame.KPts.size()>=30 ) {

				cout <<"ROBOT 1 has received a keyframe with enough keypoints" << endl << "computing BOW descriptor, saving BOW descr, kFrame, ptCls, localPose" << endl << "Run FABMAP" << endl << endl;

				if(boolSaveTestImgs) {
					// Defining strings to save imgs, files, ...
					convImgNameR1 << "testImgs/robot" << syncListenerR1.m_rID << "TestImg" << storage->testImgCtr << ".jpg";
					convMatchMatrixNameImgR1 << "testImgs/robot" << syncListenerR1.m_rID << "MatchMatrix" << storage->testImgCtr << ".jpg";
					convMatchMatrixNameFileR1 << "testImgs/robot" << syncListenerR1.m_rID << "MatchMatrix" << storage->testImgCtr << ".yml";
					imgNameR1 = convImgNameR1.str();
					matchMatrixNameImgR1 = convMatchMatrixNameImgR1.str();
					matchMatrixNameFileR1 = convMatchMatrixNameFileR1.str();
					HelperFcts::displaySaveImage("currentTestingImageRobot1", syncListenerR1.m_kFrame.img, imgNameR1);
				}


				// Compute BOW descriptor of current query image
				storage->bide->compute(syncListenerR1.m_kFrame.img, syncListenerR1.m_kFrame.KPts, syncListenerR1.m_kFrame.bowDescriptor);
				syncListenerR1.m_kFrame.fID = storage->testImgCtr;

				// Add Keyframe to storage
				storage->kFrames[storage->testImgCtr] = syncListenerR1.m_kFrame;
				// Add ptcl to storage
				storage->ptCls[storage->testImgCtr] = std::make_pair(syncListenerR1.m_rID, *(syncListenerR1.m_localPtCl) );
				syncListenerR1.m_localPtCl->clear();
				// Add keyframe pose in initial robot COS to storage
				storage->localPoses[storage->testImgCtr] = std::make_pair(syncListenerR1.m_rID,syncListenerR1.m_localPose);
				storage->localScales[storage->testImgCtr] = std::make_pair(syncListenerR1.m_rID,syncListenerR1.m_sim3LocalPoseScale);

				// RUN FABMAP ROBOT 1
				storage->testBOWDescriptors_allRobots.push_back(syncListenerR1.m_kFrame.bowDescriptor);
				storage->fabmap->compare(syncListenerR1.m_kFrame.bowDescriptor, storage->matches, true);

				if(!boolEndAfterFound1Match) {

					// SEARCH FOR BEST MATCH
					storage->searchForGoodMatch();

					// Compute transformation between matched Images and subsequently between PtCls
					if(storage->boolMatch == true) {


						/*
						//transformation_t iG;
						cout << "find iG and vis" << endl;
						storage->findTrafoInitialGuess();
						storage->findTrafo();



						pcl::PointCloud<pcl::PointXYZ>::Ptr TwoPtCls2Query (new pcl::PointCloud<pcl::PointXYZ>());
						pcl::PointCloud<pcl::PointXYZ>::Ptr TwoPtCls2Test (new pcl::PointCloud<pcl::PointXYZ>());
						pcl::PointCloud<pcl::PointXYZ>::Ptr TwoPtCls2TestTransformed (new pcl::PointCloud<pcl::PointXYZ>());

						int queryImgCtrMatch = storage->testImgCtr;
						int testImgCtrMatch = storage->matchedKeyFramesMap[queryImgCtrMatch].second;

						*TwoPtCls2Query = storage->ptCls[queryImgCtrMatch].second;
						*TwoPtCls2Test = storage->ptCls[testImgCtrMatch].second;
						TwoPtCls2Query->header.frame_id = "world";
						TwoPtCls2Query->header.stamp = 0;
						TwoPtCls2Query->height = 1;
						TwoPtCls2Test->header.frame_id = "world";
						TwoPtCls2Test->header.stamp = 0;
						TwoPtCls2Test->height = 1;

						Eigen::Matrix4f tCamQueryCamTest(Eigen::Matrix4f::Identity(4,4));
						Eigen::Matrix4f tWorldQCamQuery(Eigen::Matrix4f::Identity(4,4));
						Eigen::Matrix4f tWorldTCamTest(Eigen::Matrix4f::Identity(4,4));
						Eigen::Matrix4f tCamTestWorldT(Eigen::Matrix4f::Identity(4,4));
						Eigen::Matrix4f tWorldQCamTest(Eigen::Matrix4f::Identity(4,4));

						HelperFcts::poseStampedROSToMatrix4f(storage->localPoses[queryImgCtrMatch].second, tWorldQCamQuery);
						HelperFcts::poseStampedROSToMatrix4f(storage->localPoses[testImgCtrMatch].second, tWorldTCamTest);
						HelperFcts::invTrafo(tWorldTCamTest,tCamTestWorldT);
						tCamQueryCamTest.block(0,0,3,4)  = storage->iGMap[queryImgCtrMatch].first.block(0,0,3,4).cast<float>();

						tWorldQCamTest = tWorldQCamQuery;//*tCamQueryCamTest;


						pcl::transformPointCloud (*TwoPtCls2Test, *TwoPtCls2TestTransformed, tWorldQCamTest );

						pub1.publish(TwoPtCls2Test);
						pub2.publish(TwoPtCls2TestTransformed);
						pub3.publish(TwoPtCls2Query);




						helperTrafoToROSMsg( storage->iGMap[storage->testImgCtr].first, trafoROSMsgR1 );

						cout << "Set EndAfterFound1Match to TRUE" << endl;
						boolEndAfterFound1Match = true;
						*/
						storage->boolMatch = false;
					}
				}
				boolNewKF = true;
				storage->testImgCtr = storage->testImgCtr + 1;
			}



			if(!syncListenerR2.m_kFrame.img.empty()  && syncListenerR2.m_kFrame.KPts.size()>=30 ) {

				cout <<"ROBOT 2 has received a keyframe with enough keypoints" << endl << "computing BOW descriptor, saving BOW descr, kFrame, ptCls, localPose" << endl << "Run FABMAP" << endl << endl;

				if(boolSaveTestImgs) {
					// Defining strings to save imgs, files, ...
					convImgNameR2 << "testImgs/robot" << syncListenerR2.m_rID << "TestImg" << storage->testImgCtr << ".jpg";
					convMatchMatrixNameImgR2 << "testImgs/robot" << syncListenerR2.m_rID << "MatchMatrix" << storage->testImgCtr << ".jpg";
					convMatchMatrixNameFileR2 << "testImgs/robot" << syncListenerR2.m_rID << "MatchMatrix" << storage->testImgCtr << ".yml";
					imgNameR2 = convImgNameR2.str();
					matchMatrixNameImgR2 = convMatchMatrixNameImgR2.str();
					matchMatrixNameFileR2 = convMatchMatrixNameFileR2.str();
					HelperFcts::displaySaveImage("currentTestingImageRobot2", syncListenerR2.m_kFrame.img, imgNameR2);
				}

				// Compute BOW descriptor of current query image
				storage->bide->compute( syncListenerR2.m_kFrame.img, syncListenerR2.m_kFrame.KPts, syncListenerR2.m_kFrame.bowDescriptor );
				syncListenerR2.m_kFrame.fID = storage->testImgCtr;

				// Add keyframe to storage
				storage->kFrames[storage->testImgCtr] = syncListenerR2.m_kFrame;
				// Add ptcl to storage
				storage->ptCls[storage->testImgCtr] = std::make_pair( syncListenerR2.m_rID, *(syncListenerR2.m_localPtCl) );
				syncListenerR2.m_localPtCl->clear();
				// Add keyframe pose in initial robot COS to storage
				storage->localPoses[storage->testImgCtr] = std::make_pair(syncListenerR2.m_rID,syncListenerR2.m_localPose);
				storage->localScales[storage->testImgCtr] = std::make_pair(syncListenerR2.m_rID,syncListenerR2.m_sim3LocalPoseScale);

				// RUN FABMAP ROBOT 2
				storage->testBOWDescriptors_allRobots.push_back(syncListenerR2.m_kFrame.bowDescriptor);
				storage->fabmap->compare(syncListenerR2.m_kFrame.bowDescriptor, storage->matches, true);


				if(!boolEndAfterFound1Match) {

					// SEARCH FOR GOOD MATCHES
					storage->searchForGoodMatch();

					if(storage->boolMatch == true) {
						//transformation_t iG;
						storage->findTrafoInitialGuess();
						storage->findTrafo();







						pcl::PointCloud<pcl::PointXYZ>::Ptr TwoPtCls2Query (new pcl::PointCloud<pcl::PointXYZ>());
						pcl::PointCloud<pcl::PointXYZ>::Ptr TwoPtCls2Test (new pcl::PointCloud<pcl::PointXYZ>());
						pcl::PointCloud<pcl::PointXYZ>::Ptr TwoPtCls2QueryTransformed (new pcl::PointCloud<pcl::PointXYZ>());

						int queryImgCtrMatch = storage->testImgCtr; // Q=R2
						int testImgCtrMatch = storage->matchedKeyFramesMap[queryImgCtrMatch].second; // T=R1
						HelperFcts::saveImage(storage->kFrames[queryImgCtrMatch].img, "matchingImgs/QueryImg.jpg");
						HelperFcts::saveImage(storage->kFrames[testImgCtrMatch].img, "matchingImgs/TestImg.jpg");


						*TwoPtCls2Query = storage->ptCls[queryImgCtrMatch].second;
						*TwoPtCls2Test = storage->ptCls[testImgCtrMatch].second;
						TwoPtCls2Query->header.frame_id = "world";
						TwoPtCls2Query->header.stamp = 0;
						TwoPtCls2Query->height = 1;
						TwoPtCls2Test->header.frame_id = "world";
						TwoPtCls2Test->header.stamp = 0;
						TwoPtCls2Test->height = 1;

						Eigen::Matrix4f tCamQueryCamTest(Eigen::Matrix4f::Identity(4,4));
						Eigen::Matrix4f tCamTestCamQuery(Eigen::Matrix4f::Identity(4,4));
						Eigen::Matrix4f tWorldQCamQuery(Eigen::Matrix4f::Identity(4,4));
						Eigen::Matrix4f tCamQueryWorldQ(Eigen::Matrix4f::Identity(4,4));
						Eigen::Matrix4f tWorldTCamTest(Eigen::Matrix4f::Identity(4,4));
						Eigen::Matrix4f tCamTestWorldT(Eigen::Matrix4f::Identity(4,4));
						Eigen::Matrix4f tWorldTCamQuery(Eigen::Matrix4f::Identity(4,4));

						HelperFcts::poseStampedROSToMatrix4f(storage->localPoses[queryImgCtrMatch].second, tWorldQCamQuery);
						HelperFcts::poseStampedROSToMatrix4f(storage->localPoses[testImgCtrMatch].second, tWorldTCamTest);
						HelperFcts::invTrafo(tWorldTCamTest,tCamTestWorldT);
						HelperFcts::invTrafo(tWorldQCamQuery,tCamQueryWorldQ);
						tCamQueryCamTest.block(0,0,3,4)  = storage->iGMap[queryImgCtrMatch].first.block(0,0,3,4).cast<float>();
						HelperFcts::invTrafo(tCamQueryCamTest,tCamTestCamQuery);

						tWorldTCamQuery = tCamQueryCamTest;//tWorldTCamTest*tCamTestCamQuery*tCamQueryWorldQ;//*tCamTestCamQuery;

						helperMatrix4fToROSTrafoMsg( tCamQueryWorldQ, trafoROSMsgC2R2);
						helperMatrix4fToROSTrafoMsg( tCamTestCamQuery, trafoROSMsgC1C2);
						helperMatrix4fToROSTrafoMsg( tWorldTCamTest, trafoROSMsgR1C1);
						helperMatrix4fToROSTrafoMsg( tCamTestWorldT, trafoROSMsgC1R1);

						pcl::transformPointCloud (*TwoPtCls2Query, *TwoPtCls2QueryTransformed, tWorldTCamQuery );

						pub1.publish(TwoPtCls2Query);
						pub2.publish(TwoPtCls2QueryTransformed);
						pub3.publish(TwoPtCls2Test);






						helperTrafoToROSMsg( storage->iGMap[storage->testImgCtr].first, trafoROSMsgR2 );

						cout << "Set EndAfterFound1Match to TRUE" << endl;
						boolEndAfterFound1Match = true;
						storage->boolMatch = false;
					}
				}

				boolNewKF = true;
				storage->testImgCtr = storage->testImgCtr + 1; // testImgCtr contains nbr of testing Imgs + query Img
			}

			bool boolVisualizeLocalPtCl = false;
			if(boolNewKF == true && boolVisualizeLocalPtCl == true) {
				//storage->visualizeLocalPtCl(2, pub1, pub2, pub3);
				boolNewKF = false;
			}
			bool boolClear = false;
			if( ( ((storage->testImgCtr % 60) == 0) || ((storage->testImgCtr % 60) == 1) ) && boolClear == true && (storage->testImgCtr != 0) && (storage->testImgCtr != 1) ) {
				cout << "Clear Data" << endl;
				storage->clearData();
			}

			//helperPoseToROSTrafoMsg(syncListenerR2.m_localPose.pose,trafoROSMsgR2);
			br.sendTransform(tf::StampedTransform(trafoROSMsgC1C2, ros::Time::now(), "cam2", "cam1"));
			//br.sendTransform(tf::StampedTransform(trafoROSMsgC2R2, ros::Time::now(), "world", "cam2"));
			//br.sendTransform(tf::StampedTransform(trafoROSMsgR1C1, ros::Time::now(), "cam1", "robot1"));

			syncListenerR1.clearData();
			syncListenerR2.clearData();

			rTest.sleep();


		} // if (ros::ok())
	} // end if(doTesting)

/*
	if(doTesting) {
		Ptr<of2::FabMap> fabmap2;
		vector<of2::IMatch> matches2;
		fabmap2 = new of2::FabMap2(storage->clTree, 0.4, 0, of2::FabMap::SAMPLED | of2::FabMap::CHOW_LIU);
		fabmap2->addTraining(storage->trainBOWDescriptors_allRobots);

		CentralStorage* storage2 =  new CentralStorage(2);
		storage2->testBOWDescriptors_allRobots = storage->testBOWDescriptors_allRobots;
		storage2->kFrames = storage->kFrames;

		fabmap2->compare(storage2->testBOWDescriptors_allRobots, storage2->matches, true);

		cout << endl<<endl<< "FABMAP FINAL: " << endl;
		helperDispOutput(*storage2,true,"testImgs/robot1MatchMatrixEnd.jpg","testImgs/robot1MatchMatrixFileEnd.yml");
		waitKey(100);
	}
*/
	storage->~CentralStorage();
	cout << "END OF PROGRAM --------------------------------" << endl;
}
