#include "CentralStorage.h"

using namespace std;
using namespace opengv;


// Constructor
CentralStorage::CentralStorage(int nbrRobotsInput) :
		nbrRobots(nbrRobotsInput)
		{
	// Init camera Matrix and distortion coefficients
	//float camMatArr[3][3] = {{442.777026, 0, 291.591624},{0, 444.581889, 207.395871},{0, 0, 1}};
	//float distCoeffsArr[5] = {-0.343887, 0.100088, -0.001316, -0.000163, 0};
	//camMat = Mat(3,3,CV_32FC1,camMatArr);
	//distCoeffs = Mat(5,1,CV_32FC1,distCoeffsArr);


	nbrVisWords = 700;
	stopDataCollection = false;

	testImgCtr = 0;


	// Detector, Extractor, Matcher, BOW Img Descriptor, BOW KMeans trainer
    //detector = new DynamicAdaptedFeatureDetector(AdjusterAdapter::create("SURF"),150,250,4); //
	//detector = new DynamicAdaptedFeatureDetector(AdjusterAdapter::create("SIFT"),400,500,5); //
	detector = new SiftFeatureDetector(
	                                     0, // nFeatures
	                                     4, // nOctaveLayers
	                                     0.04, // contrastThreshold
	                                     10, //edgeThreshold
	                                     1.6 //sigma
	                                     );
	//extractor = new SurfDescriptorExtractor(500,4,2,false,true); //hessian threshold, noctave,noctavelayers,extended,upright
	extractor = new SiftDescriptorExtractor(); //hessian threshold, noctave,noctavelayers,extended,upright
	matcher = DescriptorMatcher::create("FlannBased");
	bide = 	new BOWImgDescriptorExtractor(extractor, matcher);
	// clustering (nbr clusters = nbr vis words)
	//define Term Criteria
	TermCriteria tc(CV_TERMCRIT_ITER,100,0.001);
	//retries number
	int retries=1;
	//necessary flags
	int flags=KMEANS_PP_CENTERS;
	BOWTrainer = new BOWKMeansTrainer(nbrVisWords,tc,retries,flags);



	// Initialize global Transforms with identity transform
	tf::Transform initTrafo;
	initTrafo.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
	tf::Quaternion initQ;
	initQ.setRPY(0, 0, 0);
	initTrafo.setRotation(initQ);
	for(int i=0; i<nbrRobots; ++i) {
		geometry_msgs::TransformStamped initTrafoStamped;
		initTrafoStamped.header.seq = 0;
//		initTrafoStamped.header.stamp = 0;
		initTrafoStamped.header.frame_id = "world";
		stringstream ss;
		ss << "robot" << i;
		initTrafoStamped.child_frame_id = ss.str();

		initTrafoStamped.transform.translation.x = initTrafo.getOrigin().getX();
		initTrafoStamped.transform.translation.y = initTrafo.getOrigin().getY();
		initTrafoStamped.transform.translation.z = initTrafo.getOrigin().getZ();
		initTrafoStamped.transform.rotation.x = initTrafo.getRotation().getAxis().getX();
		initTrafoStamped.transform.rotation.y = initTrafo.getRotation().getAxis().getY();
		initTrafoStamped.transform.rotation.z = initTrafo.getRotation().getAxis().getZ();
		initTrafoStamped.transform.rotation.w = initTrafo.getRotation().getW();

		globalTrafos[i] = initTrafoStamped;
	}

}



CentralStorage::~CentralStorage() {}



void CentralStorage::generateVocabulary() {

	this->BOWTrainer->add(this->trainDescriptors_allRobots);
	this->vocabulary = this->BOWTrainer->cluster(); // Returns the cluster centers (descriptors are clustered into nbr vis words)
	
}

void CentralStorage::computeTrainBOWDescriptors() {

	int imgCtr = 0;
	cv::Mat BOWDescriptors;

	std::vector<cv::Mat>::iterator itImg;
	for(itImg = this->trainImgs_allRobots.begin(); itImg != this->trainImgs_allRobots.end(); ++itImg) {
		this->bide->compute(*itImg, this->trainKPts_allRobots[imgCtr], BOWDescriptors);
		this->trainBOWDescriptors_allRobots.push_back(BOWDescriptors);
		imgCtr = imgCtr + 1;
	}

}

void CentralStorage::generateCLTree() {

	of2::ChowLiuTree treeBuilder;
	treeBuilder.add(this->trainBOWDescriptors_allRobots);
	this->clTree = treeBuilder.make();

}

#define MIN_FABMAP_MATCH 0.05 //0.97
#define MIN_MATCHES 16 //default: 6
#define MIN_NODE_CHAIN 30
#define MAX_MATCHING_DIST 10.0
#define MAX_HEADING_DIFF 0.2
void CentralStorage::searchForGoodMatch() {

	if(this->testImgCtr>0) {

		ostringstream convStringFileName, convStringContent;
		string matchedImgsFileName, matchedImgsContent;
		int kFrameNbr = -10000;

		vector<of2::IMatch>::const_iterator it;
		float maxMatch = 0.0;
		for(it = this->matches.end()-this->testImgCtr; it != this->matches.end(); ++it) {


			if(it->match < MIN_FABMAP_MATCH) {
				//cout << "match probability too low" << endl;
				continue;
			}

			if(it->imgIdx != -1) { // comparing query Img to testing Img associated with kFrameNbr
				kFrameNbr = it->imgIdx;
			}
			else if(it->imgIdx == -1) { // comparing query Img to itself
				kFrameNbr = this->kFrames.size()-1;
				continue; // Do not test to itself
			}

			if(this->kFrames[this->testImgCtr].rID == this->kFrames[kFrameNbr].rID && ((this->testImgCtr-kFrameNbr) < 7) ) { // Avoid matching of subsequent imgs from same robot
				continue;
			}

			if(this->kFrames[this->testImgCtr].rID == this->kFrames[kFrameNbr].rID) { // No matches of same robot
				continue;
			}


			convStringFileName << "matchingImgs/robot" << this->kFrames[this->testImgCtr].rID << "MatchedImgsQueryImg" << this->kFrames[this->testImgCtr].fID << ".yml";
			matchedImgsFileName = convStringFileName.str();

			convStringContent << "robot: " << this->kFrames[kFrameNbr].rID << "   MatchedImg: " << this->kFrames[kFrameNbr].fID << "   MatchProbab: " << it->match << "     ";
			matchedImgsContent = convStringContent.str();

			this->matchedKeyFrames.first = this->testImgCtr; // query Img
			this->matchedKeyFrames.second = kFrameNbr; // matched test Img
			this->matchedKeyFramesMap.insert(std::pair<int,std::pair <int,int> >(this->testImgCtr,this->matchedKeyFrames)); // store with key "nbr of query image (= this->testImgCtr)"
		} // END for loop through matches
		if(!matchedImgsContent.empty()) {
			cout << "Found a good match" << endl;
			HelperFcts::saveStringToFile(matchedImgsContent, matchedImgsFileName);
			this->boolMatch = true;
		}
	} // END if this->testImgCtr > 0
}

void CentralStorage::findTrafoInitialGuess() {

	cout << "Find initial Guess" << endl;
	int queryImgNbr = this->testImgCtr;
	int testImgNbr = matchedKeyFramesMap[queryImgNbr].second;
	Mat queryImg,testImg;
    std::vector<KeyPoint> queryImgKPts, testImgKPts;

    queryImg = this->kFrames[queryImgNbr].img;
	testImg = this->kFrames[testImgNbr].img;
	cout << "debug 01" << endl;
	queryImgKPts = this->kFrames[queryImgNbr].KPts;
	testImgKPts = this->kFrames[testImgNbr].KPts;
	cout << "debug 02" << endl;

	// Extract descriptors from imgs
	Mat queryImgDescr, testImgDescr;
	this->extractor->compute(queryImg,queryImgKPts,queryImgDescr);
	this->extractor->compute(testImg,testImgKPts,testImgDescr);
	cout << "debug 03" << endl;

	// match descriptors
	std::vector< DMatch > resultingMatches;
	cout << "debug 04" << endl;
//	this->matcher->match(queryImgDescr,testImgDescr,resultingMatches);
	this->matcher->match(queryImgDescr,testImgDescr,resultingMatches);

cout << "debug 1" << endl;
//	//derive correspondences based on random point-cloud
	bearingVector_t bearingVectorQuery,bearingVectorTest;
	bearingVectors_t bearingVectorsQuery;
	bearingVectors_t bearingVectorsTest;

	vector<Point2f> matchedPtsQuery, matchedPtsTest;
	for (int i = 0; i < resultingMatches.size(); ++i)
	{
		Point2f matchedPtQuery = queryImgKPts[resultingMatches[i].queryIdx].pt;
		Point2f matchedPtTest = testImgKPts[resultingMatches[i].trainIdx].pt;

		matchedPtsQuery.push_back(matchedPtQuery);
		matchedPtsTest.push_back(matchedPtTest);

		bearingVectorQuery[0] = matchedPtQuery.x;
		bearingVectorQuery[1] = matchedPtQuery.y;
		bearingVectorQuery[2] = 1.0;
		bearingVectorQuery.normalize();

		bearingVectorTest[0] = matchedPtTest.x;
		bearingVectorTest[1] = matchedPtTest.y;
		bearingVectorTest[2] = 1.0;
		bearingVectorTest.normalize();

		bearingVectorsQuery.push_back(bearingVectorQuery);
		bearingVectorsTest.push_back(bearingVectorTest);
	}

	cout << "debug 2" << endl;

//	//set experiment parameters
//	double noise = 0.5;
//	double outlierFraction = 0.1;
//	size_t numberPoints = 100;
//
//	//generate a random pose for viewpoint 1
//	translation_t position1 = Eigen::Vector3d::Zero();
//	rotation_t rotation1 = Eigen::Matrix3d::Identity();
//
//	//generate a random pose for viewpoint 2
//	translation_t position2 = opengv::generateRandomTranslation(2.0);
//	rotation_t rotation2 = opengv::generateRandomRotation(0.5);
//
//	//create a fake central camera
//	translations_t camOffsets;
//	rotations_t camRotations;
//	generateCentralCameraSystem( camOffsets, camRotations );
//
//	bearingVectors_t bearingVectors1;
//	bearingVectors_t bearingVectors2;
//	std::vector<int> camCorrespondences1; //unused in the central case
//	std::vector<int> camCorrespondences2; //unused in the central case
//	Eigen::MatrixXd gt(3,numberPoints);
//	generateRandom2D2DCorrespondences(position1, rotation1, position2, rotation2,camOffsets, camRotations, numberPoints, noise, outlierFraction,bearingVectors1, bearingVectors2,camCorrespondences1, camCorrespondences2, gt );
//
//
//	relative_pose::CentralRelativeAdapter adapter(bearingVectors1,bearingVectors2);

	relative_pose::CentralRelativeAdapter adapter(bearingVectorsQuery,bearingVectorsTest);
	sac::Ransac<sac_problems::relative_pose::CentralRelativePoseSacProblem> ransac;
	boost::shared_ptr<sac_problems::relative_pose::CentralRelativePoseSacProblem> relposeproblem_ptr(new sac_problems::relative_pose::CentralRelativePoseSacProblem(adapter,sac_problems::relative_pose::CentralRelativePoseSacProblem::NISTER));
	ransac.sac_model_ = relposeproblem_ptr;
	ransac.threshold_ = 2.0*(1.0 - cos(atan(sqrt(2.0)*0.5/800.0)));
	ransac.max_iterations_ = 50;
	cout << "debug 3" << endl;

	ransac.computeModel();
	transformation_t best_transformation = ransac.model_coefficients_;
	cout << "debug 4" << endl;

	cout << "best trafo: " << best_transformation << endl;

	this->iGMap[queryImgNbr] = std::make_pair(best_transformation,testImgNbr);

}

void CentralStorage::findTrafo() {

}

void CentralStorage::visualizeLocalPtCl(int robotID, ros::Publisher pub1, ros::Publisher pub2, ros::Publisher pub3) {

	pcl::PointCloud<pcl::PointXYZ>::Ptr localPtClAssembled (new pcl::PointCloud<pcl::PointXYZ>());

	typedef std::map<int,std::pair <int,pcl::PointCloud<pcl::PointXYZ> > > ptClMap;
	for (ptClMap::iterator it = this->ptCls.begin(); it!=this->ptCls.end(); ++it) {
		if( (it->second.first == robotID) ) {
			*localPtClAssembled += it->second.second;
		}
	}

	localPtClAssembled->header.frame_id = "world";
	localPtClAssembled->height = 1;

	pub1.publish(localPtClAssembled);

}

void CentralStorage::clearData() {
	this->globalTrafos.clear();
	this->kFrames.clear();
	this->localPoses.clear();
	this->matchedKeyFramesMap.clear();
	this->matches.clear();
	this->ptCls.clear();
	this->testBOWDescriptors_allRobots.release();
	this->testImgCtr = 0;
}






