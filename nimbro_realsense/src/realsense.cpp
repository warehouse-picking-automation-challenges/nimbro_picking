// ROS driver nodelet for Intel RealSense cameras
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "realsense.h"

#include <pluginlib/class_list_macros.h>

namespace nimbro_realsense
{

RealSense::RealSense()
{
}

RealSense::~RealSense()
{
	std::unique_lock<std::mutex> lock(m_activeMutex);
	if(m_active)
	{
		m_active = false;
		m_activeMutex.unlock();

		m_captureThread.join();
		m_device->stop();
	}
}

void RealSense::onInit()
{
	ros::NodeHandle nh = getPrivateNodeHandle();

	rs::log_to_console(rs::log_severity::warn);

	m_context.reset(new rs::context);

	if(m_context->get_device_count() == 0)
	{
		NODELET_ERROR("No device connected.");
		throw std::runtime_error("No device connected");
	}

	// TODO: Device selection
	m_device = m_context->get_device(0);

	NODELET_INFO("Connected device: '%s', serial %s, firmware %s",
		m_device->get_name(),
		m_device->get_serial(),
		m_device->get_firmware_version()
	);

	std::string calibURL;
	nh.param("calibration", calibURL, std::string());

	std::string rotatedCalibURL;
	nh.param("rotated_calibration", rotatedCalibURL, std::string(""));

	m_camera.reset(new Camera(getPrivateNodeHandle(), m_device, calibURL, rotatedCalibURL, false));

	m_camera->subscriptionChanged.connect(
		boost::bind(&RealSense::checkSubscribers, this)
	);
	checkSubscribers();
}

void RealSense::checkSubscribers()
{
	std::unique_lock<std::mutex> lock(m_activeMutex);

	bool needed = m_camera->hasSubscribers();

	if(needed && !m_active)
	{
		ROS_INFO("Got a subscriber, starting");
		m_device->start();
		m_captureThread = std::thread(std::bind(&RealSense::capture, this));
		m_active = true;
	}
	else if(!needed && m_active)
	{
		ROS_INFO("No subscribers left, stopping");
		m_active = false;
		lock.unlock();
		m_captureThread.join();
		m_device->stop();
	}
}

void RealSense::capture()
{
	while(1)
	{
		{
			std::lock_guard<std::mutex> lock(m_activeMutex);
			if(!m_active)
				return;
		}

		m_camera->capture();
	}
}

}

PLUGINLIB_EXPORT_CLASS(nimbro_realsense::RealSense, nodelet::Nodelet)
