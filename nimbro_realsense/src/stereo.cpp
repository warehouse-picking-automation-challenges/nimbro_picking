// Specialized driver for two SR300 cameras
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "stereo.h"

#include <pluginlib/class_list_macros.h>

namespace nimbro_realsense
{

Stereo::Stereo()
{
}

Stereo::~Stereo()
{
	std::unique_lock<std::mutex> lock(m_activeMutex);
	if(m_active)
	{
		m_active = false;
		m_activeMutex.unlock();

		m_captureThread.join();

		for(auto& cam : m_cameras)
			cam->stop();
	}
}

void Stereo::onInit()
{
	ros::NodeHandle nh = getPrivateNodeHandle();

	rs::log_to_console(rs::log_severity::warn);

	m_context.reset(new rs::context);

	std::string calibURL;
	nh.param("calibration", calibURL, std::string(""));

	std::string rotatedCalibURL;
	nh.param("rotated_calibration", rotatedCalibURL, std::string(""));

	nh.param("calibration_mode", m_calibrationMode, false);

	m_cameras.resize(2);
	std::vector<std::string> serials(2);

	if(!nh.getParam("cam1/serial", serials[0]))
		throw std::runtime_error("need cam1/serial parameter");
	if(!nh.getParam("cam2/serial", serials[1]))
		throw std::runtime_error("need cam2/serial parameter");

	NODELET_WARN("scanning...");

	// Find wanted devices
	for(int i = 0; i < m_context->get_device_count(); ++i)
	{
		rs::device* device = m_context->get_device(i);

		NODELET_INFO("connected device serial: '%s'", device->get_serial());
		fflush(stdout);
		sleep(1);

		auto it = std::find(serials.begin(), serials.end(), device->get_serial());
		if(it == serials.end())
			continue;

		int index = it - serials.begin();
		NODELET_INFO("Found camera %d", index);

		std::stringstream ss;
		ss << "cam" << index+1;

		auto cam_nh = ros::NodeHandle(nh, ss.str());
		fprintf(stderr, "new camera\n");
		m_cameras[index].reset(new Camera(cam_nh, device, calibURL, rotatedCalibURL, index == 1));
	}

	// Check if all devices are present
	bool present = true;
	for(unsigned int i = 0; i < m_cameras.size(); ++i)
	{
		if(!m_cameras[i])
		{
			NODELET_ERROR("Camera %u is not connected (serial %s)",
				i, serials[i].c_str()
			);
			present = false;
		}
	}
	if(!present)
	{
		fprintf(stderr, "at least one camera is missing\n");
		throw std::runtime_error("At least one camera is missing (see above)");
	}

	for(auto& cam : m_cameras)
		cam->subscriptionChanged.connect(boost::bind(&Stereo::checkSubscribers, this));
	checkSubscribers();
}

void Stereo::checkSubscribers()
{
	std::unique_lock<std::mutex> lock(m_activeMutex);

	bool needed = false;
	for(auto& cam : m_cameras)
	{
		if(cam->hasSubscribers())
			needed = true;
	}

	if(needed && !m_active)
	{
		ROS_INFO("Got a subscriber, starting");
		for(auto& cam : m_cameras)
			cam->start();

		m_captureThread = std::thread(std::bind(&Stereo::capture, this));
		m_active = true;
	}
	else if(!needed && m_active)
	{
		ROS_INFO("No subscribers left, stopping");
		m_active = false;
		lock.unlock();
		m_captureThread.join();

		for(auto& cam : m_cameras)
			cam->stop();
	}
}

void Stereo::capture()
{
	while(1)
	{
		{
			std::lock_guard<std::mutex> lock(m_activeMutex);
			if(!m_active)
				return;
		}

		ros::Time stamp = ros::Time::now();

		if(m_calibrationMode)
		{
			for(int i = 0; i < 10; ++i)
			{
				for(int j = 0; j < 2; ++j)
					m_cameras[j]->skipFrame();
			}

			for(int j = 0; j < 2; ++j)
					m_cameras[j]->capture(stamp);

			sleep(5);
		}
		else
		{
			for(unsigned int activeLaser = 0; activeLaser < m_cameras.size(); ++activeLaser)
			{
				// Switch laser
				for(unsigned int i = 0; i < m_cameras.size(); ++i)
				{
					m_cameras[i]->setLaserEnabled(i == activeLaser);
				}

				for(int i = 0; i < 30; ++i)
					m_cameras[activeLaser]->skipFrame();

				if(activeLaser == 0)
					stamp = ros::Time::now();

				m_cameras[activeLaser]->capture(stamp);
			}
		}
	}
}

}

PLUGINLIB_EXPORT_CLASS(nimbro_realsense::Stereo, nodelet::Nodelet)
