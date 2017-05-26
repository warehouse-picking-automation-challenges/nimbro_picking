// Singleton SRDF model cache
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef SRDF_CACHE_H
#define SRDF_CACHE_H

#include <moveit/robot_model/robot_model.h>

namespace apc_kinematics
{

boost::shared_ptr<const srdf::Model> loadSRDF();

}

#endif
