// APC control state base class
// Author: Christian Lenz <chrislenz@uni-bonn.de>

#include <apc_control/states/state.h>
#include <nimbro_fsm/impl/utils.h>

namespace apc_control
{

State::State()
{
}



State::~State()
{
}

void apc_control::State::exit()
{
	std::string name = nimbro_fsm::impl::makePrettyStateName(typeid(*this));
	auto duration = elapsedTime();

	driver()->addStateDuration(name, duration);

	ROS_INFO("Time elapsed: %f for state %s", duration.toSec(), name.c_str());
}

}
