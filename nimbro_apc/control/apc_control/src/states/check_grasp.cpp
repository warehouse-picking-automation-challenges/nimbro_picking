// Confirm that we have the object, otherwise press harder
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <apc_control/states/check_grasp.h>

#include <apc_control/states/picking/retract.h>
#include <apc_control/states/stowing/tote_retract.h>
#include <apc_control/states/picking/grasp.h>
#include <apc_control/states/stowing/tote_grasp.h>


namespace apc_control
{

CheckGrasp::CheckGrasp()
{
}

CheckGrasp::~CheckGrasp()
{
}

nimbro_fsm::StateBase* CheckGrasp::execute()
{
	if(elapsedTime() < ros::Duration(2.0))
		return this;

	if(driver()->objectWellAttached()){
		if(driver()->getIsPicking())
			return new Retract();
		else
			return new ToteRetract();
	}
	else
	{
		ROS_WARN("I don't seem to have it yet, let's try harder...");
		if(driver()->getIsPicking())
			return new Grasp(false);
		else
			return new ToteGrasp(false);
	}
}

}
