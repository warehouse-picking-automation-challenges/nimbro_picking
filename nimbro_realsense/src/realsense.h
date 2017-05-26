// ROS driver nodelet for Intel RealSense cameras
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef NIMBRO_REALSENSE_REALSENSE_H
#define NIMBRO_REALSENSE_REALSENSE_H

#include <nodelet/nodelet.h>

#include <librealsense/rs.hpp>

#include <thread>
#include <mutex>

#include "camera.h"

namespace nimbro_realsense
{

class RealSense : public nodelet::Nodelet
{
public:
	RealSense();
	virtual ~RealSense();

	virtual void onInit() override;
private:
	void checkSubscribers();
	void capture();

	std::unique_ptr<rs::context> m_context;
	rs::device* m_device = 0;

	std::mutex m_activeMutex;
	bool m_active = false;

	std::unique_ptr<Camera> m_camera;

	std::thread m_captureThread;
};

}

#endif
