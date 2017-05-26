// Scan pose state for APC
// Author: Christian Lenz <chrislenz@uni-bonn.de>
#ifndef APC_BOX_POSE_H
#define APC_BOX_POSE_H

#include <apc_control/states/move_arm.h>
#include <nimbro_fsm/fsm_ros.h>

#include <nimbro_keyframe_server/PlayMotionAction.h>

namespace apc_control
{

	class BoxPose : public MoveArm
	{

	private:

		nimbro_keyframe_server::PlayMotionGoal computeGoal() override;
		nimbro_fsm::StateBase* nextState() override;
	};


}

#endif
