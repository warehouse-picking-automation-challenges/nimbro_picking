// Shelf registration
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef SHELF_REGISTATION_H
#define SHELF_REGISTATION_H

#include <nodelet/nodelet.h>

#include <nimbro_keyframe_server/PlayMotionAction.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include <apc_shelf_registration/ShelfRegistrationAction.h>


#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsign-compare"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#pragma GCC diagnostic pop


#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <mutex>
#include <thread>
#include <condition_variable>
#include <queue>

namespace apc_shelf_registration
{

class ShelfRegistration : public nodelet::Nodelet
{
public:
	ShelfRegistration();
	virtual ~ShelfRegistration();

	virtual void onInit() override;
private:
	typedef actionlib::SimpleActionClient<nimbro_keyframe_server::PlayMotionAction> MotionClient;
	typedef actionlib::SimpleActionServer<ShelfRegistrationAction> RegistrationServer;
	typedef pcl::PointCloud<pcl::PointXYZ> Cloud;

	void execute(const ShelfRegistrationGoal::ConstPtr& goal);

	void handleCloud(const Cloud::ConstPtr& cloud);

	void tfThread();

	std::unique_ptr<MotionClient> m_motionClient;
	std::unique_ptr<RegistrationServer> m_server;

	ros::Subscriber m_sub_cloud;

	std::mutex m_cloudMutex;
	std::condition_variable m_cloudUpdate;
	Cloud::ConstPtr m_lastCloud;

	std::unique_ptr<tf::TransformListener> m_tf;

	Cloud::Ptr m_meshCloud;

	ros::Publisher m_pub_observation;
	ros::Publisher m_pub_mesh;
	ros::Publisher m_pub_aligned;

	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> m_icp;

	tf::TransformBroadcaster m_pub_tf;
	std::mutex m_transformMutex;
	tf::StampedTransform m_shelfTransform;

	ros::Publisher m_pub_marker;

	std::thread m_tfThread;
	bool m_tfThreadShouldExit = false;

	std::queue<ros::Time> m_cloudTimes;

	std::vector<Cloud::ConstPtr> m_clouds;
};

}

#endif
