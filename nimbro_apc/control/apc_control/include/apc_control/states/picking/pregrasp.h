// Pregrasp state for APC
// Author: Christian Lenz <chrislenz@uni-bonn.de>
#ifndef APC_PREGRASP_STATE_H
#define APC_PREGRASP_STATE_H

#include <apc_control/states/move_arm.h>
#include <nimbro_fsm/fsm_ros.h>

namespace apc_control
{

	class Pregrasp : public MoveArm
	{

	private:


		nimbro_keyframe_server::PlayMotionGoal computeGoal() override;
		nimbro_fsm::StateBase* nextState() override;

		nimbro_keyframe_server::PlayMotionGoal pencilCup();

	};

}


#endif
