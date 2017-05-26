// Tote Pregrasp state for APC
// Author: Christian Lenz <chrislenz@uni-bonn.de>

#include <ros/console.h>
#include <apc_control/control.h>
#include <apc_control/states/stowing/tote_pregrasp.h>
#include <apc_control/states/stowing/tote_grasp.h>
#include <apc_control/states/stowing/tote_retract.h>


namespace apc_control
{


	nimbro_keyframe_server::PlayMotionGoal TotePregrasp::computeGoal()
	{

		nimbro_keyframe_server::PlayMotionGoal goal;

		goal.motion_msg = driver()->getPregrasp()->toMsg();
		goal.motion_name="tote_pregrasp";
		goal.use_existing_motion=false;

		return goal;
	}

	nimbro_fsm::StateBase* TotePregrasp::nextState()
	{
		return new apc_control::ToteGrasp();
	}

	nimbro_fsm::StateBase* TotePregrasp::onPredictedCollision()
	{
		ROS_ERROR("Collision predicted on tote pregrasp, aborting attempt");
		return 0;
	}

	nimbro_fsm::StateBase* TotePregrasp::onProtectiveStop(const nimbro_keyframe_server::PlayMotionResultConstPtr& result)
	{
		if(result->finish_state == nimbro_keyframe_server::PlayMotionResult::REJECTED)
			return MoveArm::onProtectiveStop(result);

		ROS_ERROR("Trying to retract...");
		if(unlockProtectiveStop())
		{
			sleep(1);
			return new ToteRetract();
		}

		driver()->shutdown();
		return 0;

	}

}
