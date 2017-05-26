// Nodelet for depth filling
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef DEPTH_FILLER_DEPTH_FILLER_NODELET_H
#define DEPTH_FILLER_DEPTH_FILLER_NODELET_H

#include <nodelet/nodelet.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <sensor_msgs/CameraInfo.h>

#include <image_geometry/pinhole_camera_model.h>

#include <depth_filler/domain_transform.h>

#include <config_server/parameter.h>

namespace depth_filler
{

class DepthFillerNodelet : public nodelet::Nodelet
{
public:
	typedef pcl::PointXYZRGB Point;
	typedef pcl::PointCloud<Point> Cloud;

	DepthFillerNodelet();
	~DepthFillerNodelet();

	virtual void onInit() override;

	void handleInfo(const sensor_msgs::CameraInfo& info);

	void handlePointCloud(const Cloud::ConstPtr& cloud);
private:
	void checkSubscribers();
	void recompute();

	bool m_active = false;
	ros::Subscriber m_sub_pc;
	ros::Subscriber m_sub_camInfo;
	ros::Publisher m_pub_pc;

	Cloud::ConstPtr m_lastCloud;

	image_geometry::PinholeCameraModel m_model;
	depth_filler::DomainTransformFiller m_filler;

	config_server::Parameter<float> m_param_ne_maxDepthChange;
	config_server::Parameter<float> m_param_ne_smoothing;

	config_server::Parameter<float> m_param_shadow_threshold;
};

}

#endif
