// Tote pose state for APC
// Author: Christian Lenz <chrislenz@uni-bonn.de>

#ifndef APC_TOTE_POSE_H
#define APC_TOTE_POSE_H

#include <apc_control/states/move_arm.h>
#include <nimbro_fsm/fsm_ros.h>

#include <nimbro_keyframe_server/PlayMotionAction.h>

namespace apc_control
{

	class TotePose : public MoveArm
	{

	private:

		nimbro_keyframe_server::PlayMotionGoal computeGoal() override;
		nimbro_fsm::StateBase* nextState() override;
		nimbro_fsm::StateBase* execute() override;
		nimbro_fsm::StateBase* onProtectiveStop(const nimbro_keyframe_server::PlayMotionResultConstPtr& result) override;

		bool m_switchVacuumOff = false;

	};


}

#endif
