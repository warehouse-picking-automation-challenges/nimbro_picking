// Stow item state for APC
// Author: Christian Lenz <chrislenz@uni-bonn.de>
#ifndef APC_STOW_ITEM_H
#define APC_STOW_ITEM_H

#include <apc_control/states/move_arm.h>
#include <nimbro_fsm/fsm_ros.h>

#include <nimbro_keyframe_server/PlayMotionAction.h>

namespace apc_control
{

	class StowItem : public MoveArm
	{

	protected:

		nimbro_keyframe_server::PlayMotionGoal computeGoal() override;
		nimbro_fsm::StateBase* nextState() override;
		virtual nimbro_fsm::StateBase* onProtectiveStop(const nimbro_keyframe_server::PlayMotionResultConstPtr& result) override;
	};


}

#endif
