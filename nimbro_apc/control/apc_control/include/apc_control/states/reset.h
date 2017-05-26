// Reset state for APC
// Author: Christian Lenz <chrislenz@uni-bonn.de>

#ifndef APC_RESET_H
#define APC_RESET_H

#include <apc_control/states/move_arm.h>
#include <nimbro_fsm/fsm_ros.h>

#include <nimbro_keyframe_server/PlayMotionAction.h>

namespace apc_control
{

	class Reset : public MoveArm
	{

	private:
		void enter() override;
		nimbro_keyframe_server::PlayMotionGoal computeGoal() override;
		nimbro_fsm::StateBase* nextState() override;
	};


}

#endif
