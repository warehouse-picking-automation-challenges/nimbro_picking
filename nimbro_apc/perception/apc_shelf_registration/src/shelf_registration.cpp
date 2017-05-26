// Shelf registration
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "shelf_registration.h"

#include <pluginlib/class_list_macros.h>

#include <pcl_conversions/pcl_conversions.h>

#include <tf_conversions/tf_eigen.h>

#include <pcl/common/transforms.h>
#include <pcl/keypoints/uniform_sampling.h>

#include <pcl_ros/point_cloud.h>

#include <ros/package.h>

#include <apc_objects/apc_objects.h>

#include <visualization_msgs/Marker.h>

constexpr float VOXEL_GRID = 0.009;

namespace
{

	class ScopeTimer
	{
	public:
		ScopeTimer()
		{
			m_startTime = ros::WallTime::now();
		}

		~ScopeTimer()
		{
			ros::WallTime time = m_startTime;

			ROS_INFO("timings:");

			for(auto& pair : m_checkpoints)
			{
				ROS_INFO(" - %20s: %10.4fs", pair.first.c_str(), (pair.second - time).toSec());
				time = pair.second;
			}

			ROS_INFO("total time: %10.4fs", (ros::WallTime::now() - m_startTime).toSec());
		}

		void addCheckpoint(const std::string& label)
		{
			m_checkpoints.emplace_back(label, ros::WallTime::now());
		}
	private:
		ros::WallTime m_startTime;
		std::vector<std::pair<std::string, ros::WallTime>> m_checkpoints;
	};

}

namespace apc_shelf_registration
{

ShelfRegistration::ShelfRegistration()
{
}

ShelfRegistration::~ShelfRegistration()
{
	m_tfThreadShouldExit = true;
	m_tfThread.join();
}

void ShelfRegistration::onInit()
{
	auto nh = getPrivateNodeHandle();

	// load the shelf  and tote point cloud and set them in the feature computing objects
	{
		m_meshCloud.reset(new Cloud());
		std::string rosPackagePath = ros::package::getPath("apc_capture");

		// shelf
		std::string MeshPath = rosPackagePath + "/resource/pcd/shelf_withbackface.pcd";
		Cloud rawCloud;
		if (pcl::io::loadPCDFile(MeshPath, rawCloud) < 0)
		{
			NODELET_ERROR_STREAM("Could not load shelf mesh");
			throw std::runtime_error("Could not load shelf mesh");
		}

		// Translate to apc_objects::distRobot2Shelf (FIXME: This is a magic number)
		Eigen::Affine3f translation(Eigen::Translation3f(apc_objects::distRobot2Shelf,0,0));
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTemp (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::transformPointCloud(rawCloud, *m_meshCloud, translation);

		m_meshCloud->header.frame_id = "world";
	}

	// Filter to the points that we can see
	{
		Cloud::Ptr tmp(new Cloud);
		tmp->reserve(m_meshCloud->size());
		tmp->header = m_meshCloud->header;

		for(auto& point : *m_meshCloud)
		{
			if(point.x > apc_objects::distRobot2Shelf + 0.05)
				continue;

			if(point.z < 0.8 || point.z > 1.85)
				continue;

			tmp->push_back(point);
		}

		m_meshCloud = tmp;
	}

	// Subsample
	{
		Cloud::Ptr tmp(new Cloud);

		pcl::PointCloud<int> indices;

		pcl::UniformSampling<pcl::PointXYZ> voxgrid;
		voxgrid.setInputCloud(m_meshCloud);
		voxgrid.setRadiusSearch(VOXEL_GRID);
		voxgrid.compute(indices);

		tmp->header = m_meshCloud->header;
		tmp->reserve(indices.size());
		for(int idx : indices)
			tmp->push_back((*m_meshCloud)[idx]);

		m_meshCloud = tmp;
	}

	m_icp.setInputTarget(m_meshCloud);
	NODELET_INFO_STREAM("mesh loaded");

	// Publish marker
	{
		m_pub_marker = nh.advertise<visualization_msgs::Marker>("marker", 1, true);

		visualization_msgs::Marker marker;
		marker.header.frame_id = "registration_shelf";
		marker.action = marker.ADD;
		marker.type = marker.MESH_RESOURCE;
		marker.pose.orientation.w = 1.0;
		marker.scale.x = marker.scale.y = marker.scale.z = 1.0;
		marker.color.r = 0.8;
		marker.color.g = 0.3;
		marker.color.b = 0.3;
		marker.color.a = 1.0;
		marker.frame_locked = true;
		marker.mesh_resource = "package://apc_model/meshes/shelf_rotated.stl";

		m_pub_marker.publish(marker);
	}

	m_pub_observation = nh.advertise<Cloud>("observation", 1, true);
	m_pub_mesh = nh.advertise<Cloud>("mesh", 1, true);
	m_pub_aligned = nh.advertise<Cloud>("aligned", 1, true);

	m_tf.reset(new tf::TransformListener(nh, ros::Duration(60.0)));

	m_shelfTransform.frame_id_ = "world";
	m_shelfTransform.child_frame_id_ = "registration_shelf";
	m_shelfTransform.setIdentity();
	m_shelfTransform.setOrigin(tf::Vector3(apc_objects::distRobot2Shelf, 0, 0));

	m_tfThread = std::thread(std::bind(&ShelfRegistration::tfThread, this));

	m_motionClient.reset(
		new MotionClient(nh, "/ros_player/play_motion")
	);

	m_server.reset(
		new RegistrationServer(nh, "register", boost::bind(&ShelfRegistration::execute, this, _1), false)
	);

	m_server->start();
}

void ShelfRegistration::execute(const ShelfRegistrationGoal::ConstPtr& goal)
{
	using MotionGoal = nimbro_keyframe_server::PlayMotionGoal;

	{
		std::unique_lock<std::mutex> lock(m_cloudMutex);
		m_clouds.clear();
		while(!m_cloudTimes.empty())
			m_cloudTimes.pop();
	}

	ScopeTimer timer;

	std::vector<char> boxes{'A', 'J', 'L'};

	NODELET_INFO("Waiting for motion client");
	if(!m_motionClient->waitForServer(ros::Duration(10.0)))
	{
		NODELET_ERROR("did not appear");
		m_server->setAborted(ShelfRegistrationResult());
		return;
	}

	bool subscribed = false;
	ros::Subscriber cloudSub;

	auto observationCloud = boost::make_shared<Cloud>();
	observationCloud->header.frame_id = "world";

	timer.addCheckpoint("init");

	for(char box : boxes)
	{
		MotionGoal goal;
		{
			std::stringstream ss;
			ss << "capture_" << box;
			goal.motion_name = ss.str();
		}
		goal.use_existing_motion = true;

		auto state = m_motionClient->sendGoalAndWait(goal);

		if(state != state.SUCCEEDED)
		{
			NODELET_ERROR("Motion client reported error: %s", state.toString().c_str());
			m_server->setAborted(ShelfRegistrationResult());
			return;
		}

		timer.addCheckpoint("motion");

		sleep(2);

		timer.addCheckpoint("motion delay");

		if(!subscribed)
		{
			cloudSub = getPrivateNodeHandle().subscribe("cloud", 1, &ShelfRegistration::handleCloud, this);
			subscribed = true;
			sleep(4);
		}

		ros::Time now = ros::Time::now();
		{
			std::unique_lock<std::mutex> lock(m_cloudMutex);
			m_cloudTimes.push(now);
			NODELET_INFO("pushed %f", now.toSec());
		}

		sleep(2);

		timer.addCheckpoint("capture delay");
	}

	if(goal->early_return)
	{
		m_server->setSucceeded(ShelfRegistrationResult());
	}

	NODELET_INFO("waiting for last cloud...");
	{
		std::unique_lock<std::mutex> lock(m_cloudMutex);
		while(!m_cloudTimes.empty())
			m_cloudUpdate.wait(lock);
	}

	timer.addCheckpoint("last cloud");

	for(auto& cloud : m_clouds)
	{
		NODELET_INFO("Getting cloud transform...");
		if(!m_tf->waitForTransform("world", cloud->header.frame_id, pcl_conversions::fromPCL(cloud->header.stamp), ros::Duration(5.0)))
		{
			NODELET_ERROR("Could not wait for transform");
			m_server->setAborted(ShelfRegistrationResult());
			return;
		}

		tf::StampedTransform tfTransform;
		try
		{
			m_tf->lookupTransform("world", cloud->header.frame_id, pcl_conversions::fromPCL(cloud->header.stamp), tfTransform);
		}
		catch(tf::TransformException& e)
		{
			NODELET_ERROR("Could not lookup transform: %s", e.what());
			m_server->setAborted(ShelfRegistrationResult());
			return;
		}

		Eigen::Affine3d transform;
		tf::transformTFToEigen(tfTransform, transform);

		timer.addCheckpoint("tf");

		Cloud transformed;
		pcl::transformPointCloud(*cloud, transformed, transform.cast<float>());

		*observationCloud += transformed;
		observationCloud->header.stamp = cloud->header.stamp;

		timer.addCheckpoint("transform + append");
	}

	auto filteredObs = boost::make_shared<Cloud>();

	filteredObs->reserve(observationCloud->size());

	for(auto& point : *observationCloud)
	{
		if(!pcl::isFinite(point))
			continue;

		auto p = point.getVector3fMap();
		if(p.x() > apc_objects::distRobot2Shelf + 0.8 || p.x() < apc_objects::distRobot2Shelf - 0.8)
			continue;
		if(std::abs(p.y()) > 0.5)
			continue;
		if(p.z() < 0 || p.z() > 2.0f)
			continue;

		filteredObs->push_back(point);
	}
	filteredObs->header = observationCloud->header;

	timer.addCheckpoint("filtering");

	// Subsample
	{
		Cloud::Ptr tmp(new Cloud);

		pcl::PointCloud<int> indices;

		pcl::UniformSampling<pcl::PointXYZ> voxgrid;
		voxgrid.setInputCloud(filteredObs);
		voxgrid.setRadiusSearch(VOXEL_GRID);
		voxgrid.compute(indices);

		tmp->header = filteredObs->header;
		tmp->reserve(indices.size());
		for(int idx : indices)
			tmp->push_back((*filteredObs)[idx]);

		filteredObs = tmp;
	}

	timer.addCheckpoint("subsampling");

	m_meshCloud->header.stamp = observationCloud->header.stamp;

	m_pub_observation.publish(filteredObs);
	m_pub_mesh.publish(m_meshCloud);

	m_icp.setMaxCorrespondenceDistance(0.08);
	m_icp.setInputSource(filteredObs);

	auto icpOutput = boost::make_shared<Cloud>();
	m_icp.align(*icpOutput);

	timer.addCheckpoint("ICP");

	ROS_INFO_STREAM("has converged: " << m_icp.hasConverged());

	Eigen::Affine3d guess;
	guess = Eigen::Translation3d(apc_objects::distRobot2Shelf, 0.0, 0.0);

	Eigen::Affine3d icpTrans(m_icp.getFinalTransformation().cast<double>());

	Eigen::Affine3d newShelfTransform = icpTrans.inverse() * guess;

	newShelfTransform.translation().z() = 0;

	{
		std::unique_lock<std::mutex> lock(m_transformMutex);
		tf::transformEigenToTF(newShelfTransform, m_shelfTransform);
	}


	m_pub_aligned.publish(icpOutput);

	if(!goal->early_return)
		m_server->setSucceeded(ShelfRegistrationResult());
}

void ShelfRegistration::handleCloud(const Cloud::ConstPtr& cloud)
{
	std::unique_lock<std::mutex> lock(m_cloudMutex);

	if(m_cloudTimes.empty())
		return;

	ROS_INFO("cloud: %f, front: %f", pcl_conversions::fromPCL(cloud->header.stamp).toSec(), m_cloudTimes.front().toSec());

	while(m_lastCloud && !m_cloudTimes.empty() && pcl_conversions::fromPCL(cloud->header.stamp) > m_cloudTimes.front())
	{
		m_clouds.push_back(m_lastCloud);
		m_cloudTimes.pop();
	}

	m_lastCloud = cloud;

	m_cloudUpdate.notify_all();
}

void ShelfRegistration::tfThread()
{
	ros::Rate rate(ros::Duration(0.2));

	while(!m_tfThreadShouldExit)
	{
		{
			std::unique_lock<std::mutex> lock(m_transformMutex);
			m_shelfTransform.stamp_ = ros::Time::now();
			m_pub_tf.sendTransform(m_shelfTransform);
		}

		rate.sleep();
	}
}

}

PLUGINLIB_EXPORT_CLASS(apc_shelf_registration::ShelfRegistration, nodelet::Nodelet)
