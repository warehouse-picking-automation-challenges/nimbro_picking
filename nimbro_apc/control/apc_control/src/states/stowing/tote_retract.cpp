// Retract state for APC
// Author: Christian Lenz <chrislenz@uni-bonn.de>
#include <ros/console.h>
#include <apc_control/states/stowing/tote_retract.h>
#include <apc_control/states/stowing/stow_pose.h>
#include <apc_control/control.h>

#include <apc_control/apc_database.h>
#include <nimbro_keyframe_server/motion.h>

#include <visualization_msgs/Marker.h>
#include <eigen_conversions/eigen_msg.h>


namespace apc_control
{
	ToteRetract::ToteRetract(bool retry)
	 : m_retry(retry)
	{}

	void ToteRetract::enter()
	{
		MoveArm::enter();

		auto objectData = driver()->getCurrentItem().objectData;

		float mass = 1.0f;

		if(objectData)
			mass = objectData->mass;
		else
			ROS_WARN("No object data, cannot set payload");

		ROS_INFO("Setting PAYLOAD to %f", mass);

		driver()->setPayload(mass);
	}

	nimbro_keyframe_server::PlayMotionGoal ToteRetract::computeGoal()
	{

		driver()->setSuctionStrength(
			driver()->getCurrentItem().objectData->suctionStrength);
		nimbro_keyframe_server::PlayMotionGoal goal;
		goal.motion_msg = driver()->getRetract()->toMsg();
		goal.motion_name="retract_tote";
		goal.use_existing_motion=false;

		return goal;
	}

	nimbro_fsm::StateBase* ToteRetract::nextState()
	{
		if(!driver()->somethingOnTip() && !driver()->getSimulation())
		{
			ROS_ERROR("Lost it!");
			driver()->switchVacuum(false);
			return 0;
		}

		driver()->setItemGrasped(true);

		return new apc_control::StowPose();
	}

	nimbro_fsm::StateBase* ToteRetract::execute()
	{
		return MoveArm::execute();
	}

	void ToteRetract::exit()
	{
		State::exit();
		driver()->switchLight(0);
	}

	nimbro_fsm::StateBase* ToteRetract::onProtectiveStop(const nimbro_keyframe_server::PlayMotionResultConstPtr& result)
	{
		if(m_retry)
		{
			unlockProtectiveStop();
			return new ToteRetract(false);
		}

		return MoveArm::onProtectiveStop(result);
	}
}
