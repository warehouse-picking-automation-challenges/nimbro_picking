// Perception state for APC
// Author: Christian Lenz <chrislenz@uni-bonn.de>

#ifndef APC_CONTROL_PERCEPTION_H
#define APC_CONTROL_PERCEPTION_H

#include <geometry_msgs/PoseStamped.h>

#include <apc_control/states/state.h>
#include <nimbro_fsm/fsm_ros.h>

#include <apc_perception/ApcPerceptionAction.h>
#include <actionlib/client/simple_action_client.h>

namespace apc_control
{
	class PerceptionDelay : public State
	{
	public:
		virtual void enter() override;
		virtual nimbro_fsm::StateBase* execute() override;
	};

	class Perception : public State
	{
	typedef actionlib::SimpleActionClient<apc_perception::ApcPerceptionAction> ActionClient;
	public:
		explicit Perception(bool allItems = false);
		~Perception();

		virtual void enter() override;
		virtual nimbro_fsm::StateBase* execute() override;
		virtual void exit() override;
	private:
		ActionClient m_actionClient;
		bool m_allItems;
	};

}

#endif
