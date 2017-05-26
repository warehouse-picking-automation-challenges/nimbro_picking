// Shelf registration state for APC
// Author: Christian Lenz <chrislenz@uni-bonn.de>

#include <ros/console.h>
#include <apc_control/control.h>
#include <apc_control/states/shelf_regist.h>
#include <apc_control/states/flip_funnel.h>

#include <ros/service.h>
#include <std_srvs/Empty.h>

namespace apc_control
{

	ShelfRegist::ShelfRegist()
	 : m_actionClient("/shelf_registration/register")
	{
	}

	ShelfRegist::~ShelfRegist()
	{
	}

	void ShelfRegist::enter()
	{
		State::enter();

		if(!m_actionClient.waitForServer())
		{
			ROS_ERROR("Could not wait for action server");
			throw std::runtime_error("Could not wait for action server");
		}

		driver()->switchLight(255);

		apc_shelf_registration::ShelfRegistrationGoal goal;
		//early return if do not use the shelf transform in the next state.
		goal.early_return = !driver()->getIsPicking();
		m_actionClient.sendGoal(goal);

	}

	void ShelfRegist::exit()
	{
		State::exit();

		driver()->switchLight(0);

		if(m_actionClient.isServerConnected())
		{
			if(!m_actionClient.getState().isDone())
			{
				fprintf(stderr, "Canceling active motion goal...\n");
				m_actionClient.cancelGoal();
			}
		}
	}


	nimbro_fsm::StateBase* ShelfRegist::execute()
	{
		const actionlib::SimpleClientGoalState& state = m_actionClient.getState();

		if(state.isDone())
		{
			return new FlipFunnel();
		}
		return this;
	}


}
