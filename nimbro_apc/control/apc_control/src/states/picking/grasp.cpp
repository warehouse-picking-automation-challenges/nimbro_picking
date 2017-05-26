// Grasp state for APC
// Author: Christian Lenz <chrislenz@uni-bonn.de>

#include <ros/console.h>
#include <apc_control/control.h>
#include <apc_control/states/picking/grasp.h>
#include <apc_control/states/picking/retract.h>
#include <apc_control/states/check_grasp.h>

#include <apc_control/apc_database.h>

#include <nimbro_keyframe_server/keyframe.h>

#include <eigen_conversions/eigen_msg.h>



namespace apc_control
{
	Grasp::Grasp(bool tryAgain)
	 : m_tryAgain(tryAgain)
	{
	}

	Grasp::~Grasp()
	{
	}

	nimbro_keyframe_server::PlayMotionGoal Grasp::computeGoal()
	{

		nimbro_keyframe_server::PlayMotionGoal goal;

		if(m_tryAgain)
		{
			goal.motion_msg = driver()->getGrasp()->toMsg();
			goal.motion_name="grasp";
		}
		else
		{
			goal.motion_msg = driver()->getGrasp2()->toMsg();
			goal.motion_name="grasp2";
		}
		goal.use_existing_motion=false;

		return goal;
	}

	nimbro_fsm::StateBase* Grasp::execute()
	{
		if(driver()->objectWellAttached())
		{
			ROS_INFO("I already got it! (in Grasp state)");
			return new Retract();
		}

		return MoveArm::execute();
	}


	nimbro_fsm::StateBase* Grasp::nextState()
	{
		if(m_tryAgain){
			driver()->switchVacuum(true);
			return new CheckGrasp();
		}
		else
			return new Retract();
	}

	nimbro_fsm::StateBase* Grasp::onProtectiveStop(const nimbro_keyframe_server::PlayMotionResultConstPtr& result)
	{
		if(result->finish_state == nimbro_keyframe_server::PlayMotionResult::REJECTED)
			return MoveArm::onProtectiveStop(result);

		ROS_ERROR("Trying to retract...");
		if(unlockProtectiveStop())
		{
			driver()->switchVacuum(true);
			sleep(1);
			return new Retract();
		}

		driver()->shutdown();
		return 0;
	}
}
