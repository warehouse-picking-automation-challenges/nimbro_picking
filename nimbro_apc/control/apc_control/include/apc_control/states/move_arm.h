// Move arm state for APC
// Author: Christian Lenz <chrislenz@uni-bonn.de>

#ifndef APC_MOVE_ARM_H
#define APC_MOVE_ARM_H

#include <apc_control/states/state.h>
#include <nimbro_fsm/fsm_ros.h>

#include <nimbro_keyframe_server/PlayMotionAction.h>
#include <actionlib/client/simple_action_client.h>
#include <apc_shelf_model/dimensions.h>
#include <mutex>

namespace apc_control
{

	class MoveArm : public State
	{
	typedef actionlib::SimpleActionClient<nimbro_keyframe_server::PlayMotionAction> ActionClient;
	public:
		MoveArm();
		~MoveArm();

		virtual void enter() override;
		virtual nimbro_fsm::StateBase* execute() override;
		virtual void exit() override;
	protected:
		virtual nimbro_keyframe_server::PlayMotionGoal computeGoal()=0;
		virtual nimbro_fsm::StateBase* nextState() = 0;
		virtual nimbro_fsm::StateBase* onProtectiveStop(const nimbro_keyframe_server::PlayMotionResultConstPtr& result);
		virtual nimbro_fsm::StateBase* onPredictedCollision();
		nimbro_keyframe_server::PlayMotionFeedbackConstPtr feedback();

		bool unlockProtectiveStop();
	private:
		std::string m_motionName;
		ActionClient m_actionClient;
		unsigned int m_tries = 0;
		nimbro_keyframe_server::PlayMotionGoal m_goal;
		nimbro_keyframe_server::PlayMotionFeedbackConstPtr m_feedback;
		std::mutex m_mutex;

		void handleFeedback(const nimbro_keyframe_server::PlayMotionFeedbackConstPtr& feedback);

	};


}

#endif
