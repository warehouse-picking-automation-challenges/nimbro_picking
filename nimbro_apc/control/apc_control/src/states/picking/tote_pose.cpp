// Tote pose state for APC
// Author: Christian Lenz <chrislenz@uni-bonn.de>
#include <ros/console.h>
#include <apc_control/control.h>
#include <apc_control/states/picking/tote_pose.h>

namespace apc_control
{

	nimbro_keyframe_server::PlayMotionGoal TotePose::computeGoal()
	{
		nimbro_keyframe_server::PlayMotionGoal goal;

		goal.motion_name = "tote_release";
		goal.motion_msg = driver()->getToteRelease()->toMsg();
		goal.use_existing_motion=false;

		return goal;
	}

	nimbro_fsm::StateBase* TotePose::nextState()
	{
		if(driver()->somethingOnTip())
		{
			driver()->setItemPlaced(true);
		}
		driver()->switchVacuum(false);
		return 0;
	}

	nimbro_fsm::StateBase* apc_control::TotePose::execute()
	{
// 		auto fb = feedback();
// 		if(fb && fb->estimated_duration < ros::Duration(1.0).toSec() && !m_switchVacuumOff)
// 		{
// 			m_switchVacuumOff = true;
// 			driver()->switchVacuum(false);
// 		}

		return MoveArm::execute();
	}


	nimbro_fsm::StateBase* TotePose::onProtectiveStop(const nimbro_keyframe_server::PlayMotionResultConstPtr&)
	{
		ROS_ERROR("Protective stop while trying to place object.");
		if(unlockProtectiveStop())
		{
			sleep(1);
			driver()->switchVacuum(false);
			return 0;
		}

		driver()->shutdown();
		return 0;
	}
}
