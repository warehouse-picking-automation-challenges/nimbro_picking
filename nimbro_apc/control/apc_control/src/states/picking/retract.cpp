// Retract state for APC
// Author: Christian Lenz <chrislenz@uni-bonn.de>
#include <ros/console.h>
#include <apc_control/states/picking/retract.h>
#include <apc_control/states/picking/tote_pose.h>
#include <apc_control/states/scan_pose.h>
#include <apc_control/control.h>

#include <apc_control/apc_database.h>
#include <nimbro_keyframe_server/keyframe.h>

#include <visualization_msgs/Marker.h>
#include <eigen_conversions/eigen_msg.h>


namespace apc_control
{
	Retract::Retract(bool retry)
	 : m_retry(retry)
	{}

	void Retract::enter()
	{
		MoveArm::enter();
		driver()->setSuctionStrength(
			driver()->getCurrentItem().objectData->suctionStrength
		);
	}

	nimbro_keyframe_server::PlayMotionGoal Retract::computeGoal()
	{

		nimbro_keyframe_server::PlayMotionGoal goal;

		goal.motion_msg = driver()->getRetract()->toMsg();
		goal.motion_name="retract";
		goal.use_existing_motion=false;

		return goal;
	}

	nimbro_fsm::StateBase* Retract::nextState()
	{
		if(!driver()->somethingOnTip() && !driver()->getSimulation())
		{
			ROS_ERROR("Lost it!");
			driver()->switchVacuum(false);
			return 0;
		}

		driver()->setItemGrasped(true);
		return new apc_control::TotePose();
	}

	nimbro_fsm::StateBase* Retract::execute()
	{
		return MoveArm::execute();
	}

	void Retract::exit()
	{
		State::exit();
		driver()->switchLight(0);
	}

	nimbro_fsm::StateBase* Retract::onProtectiveStop(const nimbro_keyframe_server::PlayMotionResultConstPtr& result)
	{
		if(m_retry)
		{
			unlockProtectiveStop();
			return new Retract(false);
		}

		return MoveArm::onProtectiveStop(result);
	}
}
