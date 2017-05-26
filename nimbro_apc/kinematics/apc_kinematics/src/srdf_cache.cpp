// Singleton SRDF model cache
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "srdf_cache.h"

#include <moveit/robot_model_loader/robot_model_loader.h>

#include <boost/thread/mutex.hpp>
#include <boost/thread/lock_guard.hpp>

#include <ros/console.h>

namespace apc_kinematics
{

static boost::shared_ptr<robot_model_loader::RobotModelLoader> g_loader;
static boost::mutex g_mutex;

boost::shared_ptr<const srdf::Model> loadSRDF()
{
	boost::unique_lock<boost::mutex> lock(g_mutex);

	if(!g_loader)
	{
		ROS_INFO("new loader");
		g_loader.reset(new robot_model_loader::RobotModelLoader("robot_description", false));
	}

	return g_loader->getSRDF();
}

}
