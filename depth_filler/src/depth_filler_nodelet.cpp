// Nodelet for depth filling
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "depth_filler_nodelet.h"

#include <pluginlib/class_list_macros.h>

#include <pcl/features/integral_image_normal.h>

#include <pcl/filters/shadowpoints.h>

#include <chrono>

namespace
{
	typedef std::chrono::high_resolution_clock clock;

	inline int milliseconds(clock::duration duration)
	{ return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count(); }
}

namespace depth_filler
{

constexpr bool DUMP_CLOUDS = false;

DepthFillerNodelet::DepthFillerNodelet()
 : m_param_ne_maxDepthChange("ne/max_depth_change", 0.0, 0.001, 10.0, 0.1)
 , m_param_ne_smoothing("ne/smoothing", 0.0, 1.0, 100.0, 10.0)
 , m_param_shadow_threshold("shadow/threshold", 0.0, 0.01, 1.0, 0.1)
{

	auto cb = boost::bind(&DepthFillerNodelet::recompute, this);
	m_param_ne_maxDepthChange.setCallback(cb);
	m_param_ne_smoothing.setCallback(cb);
	m_param_shadow_threshold.setCallback(cb);
}

DepthFillerNodelet::~DepthFillerNodelet()
{
}

void DepthFillerNodelet::checkSubscribers()
{
	bool needed = m_pub_pc.getNumSubscribers() != 0;

	if(!m_active && needed)
	{
		NODELET_INFO("Got a subscriber, starting");
		ros::NodeHandle nh = getPrivateNodeHandle();
		m_sub_pc = nh.subscribe("input", 1, &DepthFillerNodelet::handlePointCloud, this);

		m_active = true;
	}
	else if(m_active && !needed)
	{
		NODELET_INFO("No subscribers left, stopping");
		m_sub_pc.shutdown();

		m_active = false;
	}
}

void DepthFillerNodelet::onInit()
{
	auto& nh = getPrivateNodeHandle();

	auto connectCB = boost::bind(&DepthFillerNodelet::checkSubscribers, this);

	m_sub_camInfo = nh.subscribe("info", 1, &DepthFillerNodelet::handleInfo, this);
	m_pub_pc = nh.advertise<Cloud>("output", 1, connectCB, connectCB);
}

void DepthFillerNodelet::handleInfo(const sensor_msgs::CameraInfo& info)
{
	m_model.fromCameraInfo(info);
	m_sub_camInfo.shutdown();
}

void DepthFillerNodelet::recompute()
{
	if(!m_lastCloud)
		return;

	handlePointCloud(m_lastCloud);
}

void DepthFillerNodelet::handlePointCloud(const Cloud::ConstPtr& cloud)
{
	if(!m_model.initialized())
		return;

	m_lastCloud = cloud;

	if(!cloud->isOrganized())
	{
		NODELET_ERROR("Input cloud is not organized");
		return;
	}

	Cloud::Ptr filled(new Cloud);

	float scale = (float)cloud->width / m_model.fullResolution().width;
	float fx = scale * m_model.fx();
	float fy = scale * m_model.fy();
	float cx = scale * m_model.cx();
	float cy = scale * m_model.cy();

	auto start = clock::now();
	m_filler.fillCloud(*cloud, *filled, fx, fy, cx, cy);
	auto end = clock::now();
	NODELET_DEBUG("Depth fill-in took %d ms", milliseconds(end - start));

	if(DUMP_CLOUDS)
	{
		std::stringstream ss;
		ss << "/tmp/" << cloud->header.stamp;
		auto stem = ss.str();

		pcl::io::savePCDFileBinary(stem + "_unfilled.pcd", *cloud);
		pcl::io::savePCDFileBinary(stem + "_filled.pcd", *filled);
	}

	// Shadow points filter
	Cloud::Ptr output(new Cloud);
	{
		NODELET_DEBUG("shadow filter...");
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

		pcl::IntegralImageNormalEstimation<Point, pcl::Normal> ne;
		ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
		ne.setMaxDepthChangeFactor(m_param_ne_maxDepthChange());
		ne.setNormalSmoothingSize(m_param_ne_smoothing());
		ne.setInputCloud(filled);
		ne.compute(*normals);

		pcl::ShadowPoints<Point, pcl::Normal> shadow;
		shadow.setNormals(normals);
		shadow.setInputCloud(filled);
		shadow.setKeepOrganized(true);
		shadow.setThreshold(m_param_shadow_threshold());
		shadow.filter(*output);

		output->header = cloud->header;
		output->width = filled->width;
		output->height = filled->height;
	}

	m_pub_pc.publish(output);
}

}

PLUGINLIB_EXPORT_CLASS(depth_filler::DepthFillerNodelet, nodelet::Nodelet)
