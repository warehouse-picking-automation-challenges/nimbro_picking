// Single camera ROS publisher
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef NIMBRO_REALSENSE_CAMERA_H
#define NIMBRO_REALSENSE_CAMERA_H

#include <librealsense/rs.hpp>

#include <image_transport/image_transport.h>

#include <boost/signals2.hpp>

#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>

namespace nimbro_realsense
{

class Camera
{
public:
	Camera(const ros::NodeHandle& nh, rs::device* device, const std::string& calibURL, const std::string& rotatedCalibURL, bool upper);
	~Camera();

	void skipFrame();
	void capture(ros::Time stamp = ros::Time(0));

	void start();
	void stop();

	void setLaserEnabled(bool enabled);

	bool hasSubscribers();
	boost::signals2::signal<void()> subscriptionChanged;

	sensor_msgs::ImageConstPtr lastColorImage() const
	{ return m_lastImage; }
private:
	ros::NodeHandle m_nh;
	rs::device* m_device;

	std::unique_ptr<image_transport::ImageTransport> m_it;
	image_transport::CameraPublisher m_pub_color;
	image_transport::Publisher m_pub_depth;
	ros::Publisher m_pub_pointcloud;

	std::unique_ptr<image_transport::ImageTransport> m_rotated_it;
	image_transport::CameraPublisher m_pub_rotated_color;

	sensor_msgs::ImageConstPtr m_lastImage;

	std::string m_frame_id;

	bool m_upper;
	camera_info_manager::CameraInfoManager m_infoManager;
	camera_info_manager::CameraInfoManager m_rotated_infoManager;
};

}

#endif
