// Tote Grasp state for APC
// Author: Christian Lenz <chrislenz@uni-bonn.de>

#include <ros/console.h>
#include <apc_control/control.h>
#include <apc_control/states/stowing/tote_grasp.h>
#include <apc_control/states/stowing/tote_retract.h>
#include <apc_control/states/check_grasp.h>

#include <apc_control/apc_database.h>

#include <nimbro_keyframe_server/keyframe.h>

#include <eigen_conversions/eigen_msg.h>



namespace apc_control
{
	ToteGrasp::ToteGrasp(bool tryAgain)
	 : m_tryAgain(tryAgain)
	{
	}

	ToteGrasp::~ToteGrasp()
	{
	}

	nimbro_keyframe_server::PlayMotionGoal ToteGrasp::computeGoal()
	{
		nimbro_keyframe_server::PlayMotionGoal goal;

		if(m_tryAgain)
		{
			goal.motion_msg = driver()->getGrasp()->toMsg();
			goal.motion_name="tote_grasp";
		}
		else
		{
			goal.motion_msg = driver()->getGrasp2()->toMsg();
			goal.motion_name="tote_grasp2";
		}
		goal.use_existing_motion=false;

		return goal;
	}


	nimbro_fsm::StateBase* ToteGrasp::nextState()
	{
		if(m_tryAgain){
			driver()->switchVacuum(true);
			return new CheckGrasp();
		}
		else
			return new ToteRetract();
	}

	nimbro_fsm::StateBase* ToteGrasp::execute()
	{
		if(driver()->objectWellAttached())
		{
			ROS_INFO("I already got it! (in ToteGrasp state)");
			return new ToteRetract();
		}

		return MoveArm::execute();
	}

	nimbro_fsm::StateBase* ToteGrasp::onProtectiveStop(const nimbro_keyframe_server::PlayMotionResultConstPtr& result)
	{
		if(result->finish_state == nimbro_keyframe_server::PlayMotionResult::REJECTED)
			return MoveArm::onProtectiveStop(result);

		ROS_ERROR("Trying to retract...");
		if(unlockProtectiveStop())
		{
			driver()->switchVacuum(true);
			sleep(1);
			return new ToteRetract();
		}

		driver()->shutdown();
		return 0;
	}


}
