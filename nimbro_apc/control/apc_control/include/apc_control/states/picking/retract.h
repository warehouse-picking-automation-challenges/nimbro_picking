// Retract state for APC
// Author: Christian Lenz <chrislenz@uni-bonn.de>

#ifndef APC_RETRACT_STATE_H
#define APC_RETRACT_STATE_H

#include <apc_control/states/move_arm.h>
#include <nimbro_fsm/fsm_ros.h>

namespace apc_control
{

	class Retract : public MoveArm
	{
	public:
		explicit Retract(bool retry = true);

		nimbro_fsm::StateBase* execute() override;

		virtual void enter() override;
		virtual void exit() override;
	private:
		nimbro_keyframe_server::PlayMotionGoal computeGoal() override;
		nimbro_fsm::StateBase* nextState() override;
		nimbro_fsm::StateBase* onProtectiveStop(const nimbro_keyframe_server::PlayMotionResultConstPtr& result) override;

		bool m_retry;
	};

}


#endif
