// Scan pose state for APC
// Author: Christian Lenz <chrislenz@uni-bonn.de>

#include <ros/console.h>
#include <apc_control/control.h>
#include <apc_control/states/flip_funnel.h>

namespace apc_control
{
	nimbro_keyframe_server::PlayMotionGoal FlipFunnel::computeGoal()
	{
		nimbro_keyframe_server::PlayMotionGoal goal;

		goal.motion_name="flip_funnel";
		goal.use_existing_motion=true;
		return goal;
	}

	nimbro_fsm::StateBase* FlipFunnel::nextState()
	{
		return 0;
	}


}