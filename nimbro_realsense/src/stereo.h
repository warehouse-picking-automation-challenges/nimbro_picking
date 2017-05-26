// Specialized driver for two SR300 cameras
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef NIMBRO_REALSENSE_STEREO_H
#define NIMBRO_REALSENSE_STEREO_H

#include <nodelet/nodelet.h>

#include <librealsense/rs.hpp>

#include <thread>
#include <mutex>

#include "camera.h"

namespace nimbro_realsense
{

class Stereo : public nodelet::Nodelet
{
public:
	Stereo();
	virtual ~Stereo();

	virtual void onInit() override;
private:
	void checkSubscribers();
	void capture();

	std::unique_ptr<rs::context> m_context;

	std::mutex m_activeMutex;
	bool m_active = false;

	std::vector<std::shared_ptr<Camera>> m_cameras;

	std::thread m_captureThread;

	std::mutex m_apiMutex;

	bool m_calibrationMode;
};

}

#endif
