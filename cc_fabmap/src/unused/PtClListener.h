#ifndef PTCLLISTENER_H
#define PTCLLISTENER_H



#include <ros/ros.h>
//#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
//#include <pcl/conversions.h>
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>


#include "CentralStorage.h"


using namespace std;


/*! \class PtClListener
 * \brief Class listening to incoming PtCl, each instance of the class listens to one robot
 */
class PtClListener
{
public:

	int robotID;
	string topicName;
	ros::NodeHandle nh;
	ros::Subscriber subPtCl2;
	pcl::PointCloud<pcl::PointXYZ> localPtCl;


	/*! \fn const char PtClListener::PtClListener()
	* \brief Constructor initializing member variables
	*/
//	PtClListener(ros::NodeHandle _nh);
	PtClListener(int rIDInput, string topicNameInput, ros::NodeHandle nhInput);
	~PtClListener(); ///< destructor

	/*! \fn const char PtClListener::callback_ptClListener(const sensor_msgs::ImageConstPtr& msg, CentralStorage* storage)
	* \brief Member function listening to incoming PtCls
	*/
	//void callback_ptCl2Listener(const sensor_msgs::PointCloud2ConstPtr& inputPtCl, CentralStorage* storage);
	void callback_ptCl2Listener(const sensor_msgs::PointCloud2ConstPtr& inputPtCl);

};


#endif

