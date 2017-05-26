// Tote Pregrasp state for APC
// Author: Christian Lenz <chrislenz@uni-bonn.de>
#ifndef APC_TOTE_PREGRASP_STATE_H
#define APC_TOTE_PREGRASP_STATE_H

#include <apc_control/states/move_arm.h>
#include <nimbro_fsm/fsm_ros.h>

namespace apc_control
{

	class TotePregrasp : public MoveArm
	{
	protected:
		nimbro_keyframe_server::PlayMotionGoal computeGoal() override;
		nimbro_fsm::StateBase* nextState() override;
		nimbro_fsm::StateBase* onPredictedCollision() override;
		nimbro_fsm::StateBase* onProtectiveStop(const nimbro_keyframe_server::PlayMotionResultConstPtr& result);
	};

}


#endif
