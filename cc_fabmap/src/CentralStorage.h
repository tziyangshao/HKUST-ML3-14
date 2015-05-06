#ifndef CENTRALSTORAGE_H
#define CENTRALSTORAGE_H


#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/contrib/contrib.hpp>
#include "opencv2/nonfree/nonfree.hpp"
#include <algorithm>
#include <opengv/relative_pose/methods.hpp>
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/relative_pose/CentralRelativePoseSacProblem.hpp>
#include <cstdio>

#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>
#include <image_geometry/pinhole_camera_model.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
//#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include "HelperFcts.h"

using namespace std;
using namespace cv;
using namespace opengv;

/**
 * Structure containing all information concerning to one Frame
 */
struct keyFrame {
	Mat img;
	Mat imgRectGray;
	int rID;
	int fID;
	vector<KeyPoint> KPts;
	Mat bowDescriptor;
};


/*! \class CentralStorage
 * \brief Class storing all images of all robots, global maps (pose graph and point cloud)
 */
class CentralStorage {

public:

	int nbrRobots;
	image_geometry::PinholeCameraModel camModel;
	//Mat camMat;
	//Mat distCoeffs;

	// Variables
	bool stopDataCollection;
	cv::Mat trainDescriptors_allRobots;
	vector<vector<KeyPoint> > trainKPts_allRobots; // KPts are in same order as trainImgs
	vector<cv::Mat> trainImgs_allRobots; // KPts are in same order as trainImgs
	int nbrVisWords;
	cv::Mat vocabulary;
	cv::Mat trainBOWDescriptors_allRobots;
	cv::Mat clTree;




	BOWKMeansTrainer* BOWTrainer;
	Ptr<FeatureDetector> detector;
	Ptr<DescriptorExtractor> extractor;
	Ptr<DescriptorMatcher> matcher;
	BOWImgDescriptorExtractor* bide;
	Ptr<of2::FabMap> fabmap;

	/// kFrames stores the keyFrame Image, its identity number (fID=ImgCtr), the associated robot (rID), the kPts and the bowDescriptor of the keyFrame
	std::map<int,keyFrame> kFrames;

	/// the key for accessing a ptCls corresponds to the the kFrames identity number (ImgCtr), the std::pair contains the associated rID and the actual PtCl
	std::map<int,std::pair <int,pcl::PointCloud<pcl::PointXYZ> > > ptCls; // map key: ImgCtr, std::pair<int,pcl>: int -> robot ID, pcl -> pcl
	//std::map< int, pcl::PointCloud<pcl::PointXYZ> > ptClsOnly; // map key: ImgCtr, std::pair<int,pcl>: int -> robot ID, pcl -> pcl

	/// the key for accessing a local Poses (poses of the keyframe concerning the initial pose of the associated robot)
	/// corresponds to the the kFrames identity number (ImgCtr), the std::pair contains the associated rID and the actual Transformation
	std::map<int,std::pair <int,geometry_msgs::PoseStamped> > localPoses;
	/// Saves the scales for all robots and all images. IMPORTANT NOTE: only the latest scale is needed for each robot!!!! (current storage->testImgCtr!!!) <-- Maybe not true (see LSD SLAM 3.5 scale for each keyframe)
	std::map<int,std::pair <int,float> > localScales;

	/// the key for accessing a global Transformations (transformation of the robot COS concerning the world COS)
	/// corresponds to the the rID, the map contains the transformation of the associated robot relative to the world COS
	/// NOTE: The world COS is identical with the COS of the robot with rID 1!!!!!!!!!!
	std::map<int,geometry_msgs::TransformStamped> globalTrafos;


	cv::Mat testBOWDescriptors_allRobots;
	int testImgCtr;
	vector<of2::IMatch> matches;

	bool boolMatch; // registers if there is a match/ "loop closure"
	std::pair <int,int> matchedKeyFrames; // First keyFrame Number is always associated to queryImg and is equal to the access key of the std::map matchedKeyFramesMap

	/// key is the nbr of the queryImg (ImgCtr) --> access kFrame: storage->kFrames[ matchedKeyFramesMap[nbr].first ], storage->kFrames[ matchedKeyFramesMap[nbr].second ]
	std::map<int,std::pair <int,int> > matchedKeyFramesMap;
	/// key is the nbr of the queryImg (ImgCtr), iGMap.first: queryImg (ImgCtr), iGMap.second.first:  IG between 2 keyframes, iGMap.second.second: testImg (testImgCtr of matched kFrame)
	std::map<int,std::pair <opengv::transformation_t, int> > iGMap;


	/*! \fn const char CentralStorage::CentralStorage()
	* \brief Constructor setting initial values to member variables and initializing detector, extractor, matcher and BOWKMeansTrainer
	*/
	CentralStorage(int nbrRobotsInput);
	~CentralStorage();




	/*! \fn const char CentralStorage::generateVocabulary()
	* \brief Member function generating vocabulatory (dictionary/ visual words/ ...) and outputting it in "vocab.yml", also saves all descriptors (SURF) of all robots in "trainDescriptors_allRobots.yml"
	*/
	void generateVocabulary();

	void computeTrainBOWDescriptors();

	/*! \fn const char CentralStorage::generateCLTree()
	* \brief Member function generating chow liu tree and outputting it in "cltree.yml"
	*/
	void generateCLTree();

	void searchForGoodMatch();

	void findTrafoInitialGuess();

	void findTrafo();

	void visualizeLocalPtCl(int robotID, ros::Publisher pub1, ros::Publisher pub2, ros::Publisher pub3);

	void clearData();

};





#endif
