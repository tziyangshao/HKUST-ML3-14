#ifndef HELPERFCTS_H
#define HELPERFCTS_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>

#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>




using namespace std;
using namespace cv;

class HelperFcts {

public:

	HelperFcts();
	~HelperFcts();

	static void displayImage(string windowName, Mat img);
	static void displayImageKPts(string windowName, Mat img, vector<KeyPoint> kPts);
	static void displaySaveImage(string windowName, Mat img, string nameInclingDir);
	static void saveImage(Mat img, string nameInclingDir);
	static void saveMatrix(Mat matrix, string nameInclingDir);
	static void saveStringToFile(string content, string nameInclingDir);
	static void invTrafo(Eigen::Matrix4f trafo, Eigen::Matrix4f & trafoInv);
	static void trafoFloat7ToQuatTranslScale(float simTrafo[7], Eigen::Quaternion<float> & quat, Eigen::Translation<float,3> & transl, float & sc);
	static void trafoFloat7Matrix4f(float trafoFloat[7], Eigen::Matrix4f & trafoMat);
	static void getInvQuatTranslScale(Eigen::Quaternion<float> quat, Eigen::Quaternion<float> & quatInv, Eigen::Translation<float,3> transl, Eigen::Translation<float,3> & translInv, float sc, float & scInv);
	static void eigenQuatTranslPoseToROSPose(Eigen::Quaternion<float> quat, Eigen::Translation<float,3> transl, geometry_msgs::PoseStamped & pose);
	static void poseStampedROSToMatrix4f(geometry_msgs::PoseStamped poseROS, Eigen::Matrix4f & tCamQueryCamTest);


};










#endif

