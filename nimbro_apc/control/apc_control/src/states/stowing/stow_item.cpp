// Scan pose state for APC
// Author: Christian Lenz <chrislenz@uni-bonn.de>

#include <ros/console.h>
#include <apc_control/control.h>
#include <apc_control/states/stowing/stow_item.h>
#include <apc_control/states/stowing/home_pose.h>

#include <nimbro_keyframe_server/motion.h>
#include <nimbro_keyframe_server/keyframe.h>

namespace apc_control
{

	nimbro_keyframe_server::PlayMotionGoal StowItem::computeGoal()
	{
		nimbro_keyframe_server::PlayMotionGoal goal;

		goal.motion_msg = driver()->getStowPlace()->toMsg();
		goal.motion_name="stow_item";
		goal.use_existing_motion=false;

		return goal;
	}

	nimbro_fsm::StateBase* StowItem::nextState()
	{
		driver()->switchVacuum(false);
		return new HomePose();
	}

	nimbro_fsm::StateBase* StowItem::onProtectiveStop(const nimbro_keyframe_server::PlayMotionResultConstPtr& result)
	{
		if(result->finish_state == nimbro_keyframe_server::PlayMotionResult::REJECTED)
			return MoveArm::onProtectiveStop(result);

		ROS_ERROR("Trying to retract...");
		if(unlockProtectiveStop())
		{
			driver()->switchVacuum(false);
			sleep(1);
			return new HomePose();
		}

		driver()->shutdown();
		return 0;
	}

}
