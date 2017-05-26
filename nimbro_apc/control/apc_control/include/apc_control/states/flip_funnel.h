// Flip funnel state for APC
// Author: Christian Lenz <chrislenz@uni-bonn.de>
#ifndef APC_ROTATE_ARM_H
#define APC_ROTATE_ARM_H

#include <apc_control/states/move_arm.h>
#include <nimbro_fsm/fsm_ros.h>

#include <nimbro_keyframe_server/PlayMotionAction.h>

namespace apc_control
{

	class FlipFunnel : public MoveArm
	{
	private:

		nimbro_keyframe_server::PlayMotionGoal computeGoal() override;
		nimbro_fsm::StateBase* nextState() override;

	};


}

#endif
