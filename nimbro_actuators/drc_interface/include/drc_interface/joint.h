// Joint subclass containing hw information
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef MOMARO_JOINT_H
#define MOMARO_JOINT_H

#include <robotcontrol/model/joint.h>
#include <config_server/parameter.h>

#include <stdint.h>

namespace drc_interface
{

class ControllerLink;

struct Joint : public robotcontrol::Joint
{
	typedef boost::shared_ptr<Joint> Ptr;

	explicit Joint(const std::string& name);

	config_server::Parameter<int> id;
	config_server::Parameter<int> encoderTicks;
	config_server::Parameter<int> maxTorque;
	config_server::Parameter<bool> isTTL;
	config_server::Parameter<int> offset;
	config_server::Parameter<bool> invert;
	config_server::Parameter<float> NmPerAmp;
	config_server::Parameter<bool> velocityMode;
	config_server::Parameter<float> gearboxRatio;
	config_server::Parameter<int> maxVelocity;
	config_server::Parameter<int> fadeInVelocity;

	ControllerLink* link;
	
	double temperature;
	unsigned int timeouts;
	uint8_t error;
	uint16_t voltage;
	int goalTorque;

	double timeoutConfidence;
};

}

#endif
