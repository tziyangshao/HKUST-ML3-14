#ifndef POSELISTENER_H
#define POSELISTENER_H



#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>



#include "CentralStorage.h"


using namespace std;


/*! \class PoseListener
 * \brief Class listening to incoming PtCl, each instance of the class listens to one robot
 */
class PoseListener
{
public:

	int robotID;
	string topicName;
	ros::NodeHandle nh;
	ros::Subscriber subPose;
	geometry_msgs::PoseStamped pose;


	/*! \fn const char PtClListener::PtClListener()
	* \brief Constructor initializing member variables
	*/
//	PtClListener(ros::NodeHandle _nh);
	PoseListener(int rIDInput, string topicNameInput, ros::NodeHandle nhInput);
	~PoseListener(); ///< destructor

	/*! \fn const char PtClListener::callback_ptClListener(const sensor_msgs::ImageConstPtr& msg, CentralStorage* storage)
	* \brief Member function listening to incoming PtCls
	*/
	void callback_poseListener(const geometry_msgs::PoseStamped::ConstPtr& inputPose);

};


#endif

