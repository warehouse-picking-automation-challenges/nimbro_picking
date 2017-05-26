// Tote Grasp state for APC
// Author: Christian Lenz <chrislenz@uni-bonn.de>

#ifndef APC_TOTE_GRASP_STATE_H
#define APC_TOTE_GRASP_STATE_H

#include <apc_control/states/move_arm.h>
#include <nimbro_fsm/fsm_ros.h>



#include <nimbro_keyframe_server/PlayMotionAction.h>
#include <geometry_msgs/PoseStamped.h>

namespace apc_control
{

	class ToteGrasp : public MoveArm
	{
	public:
		ToteGrasp(bool tryAgain = true);
		virtual ~ToteGrasp();

	protected:
		nimbro_keyframe_server::PlayMotionGoal computeGoal() override;
		nimbro_fsm::StateBase* nextState() override;
		nimbro_fsm::StateBase* execute() override;
		virtual nimbro_fsm::StateBase* onProtectiveStop(const nimbro_keyframe_server::PlayMotionResultConstPtr& result) override;
	private:
		bool m_tryAgain;
	};

}


#endif
