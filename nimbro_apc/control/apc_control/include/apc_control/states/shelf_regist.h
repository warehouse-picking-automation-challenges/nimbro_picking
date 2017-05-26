// Shelf registration state for APC
// Author: Christian Lenz <chrislenz@uni-bonn.de>

#ifndef APC_SHELF_REGISTRATION_H
#define APC_SHELF_REGISTRATION_H

#include <apc_control/states/state.h>
#include <nimbro_fsm/fsm_ros.h>

#include <actionlib/client/simple_action_client.h>
#include <apc_shelf_registration/ShelfRegistrationAction.h>
#include <mutex>

namespace apc_control
{

	class ShelfRegist : public State
	{
	typedef actionlib::SimpleActionClient<apc_shelf_registration::ShelfRegistrationAction> ActionClient;
	public:
		ShelfRegist();
		~ShelfRegist();

		virtual void enter() override;
		virtual nimbro_fsm::StateBase* execute() override;
		virtual void exit() override;
	private:
		ActionClient m_actionClient;

	};


}

#endif
