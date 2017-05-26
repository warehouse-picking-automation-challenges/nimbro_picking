// Home state for APC
// Author: Christian Lenz <chrislenz@uni-bonn.de>

#include <ros/console.h>
#include <apc_control/control.h>
#include <apc_control/states/reset.h>

#include <nimbro_keyframe_server/motion.h>
#include <nimbro_keyframe_server/keyframe.h>

namespace apc_control
{

	void Reset::enter()
	{
		MoveArm::enter();
	}

	nimbro_keyframe_server::PlayMotionGoal Reset::computeGoal()
	{
		namespace kfs = nimbro_keyframe_server;


		kfs::Motion motion;
		motion.setName("reset motion");
		kfs::KeyFrame kf;
		kf.setLabel("reset_motion");
		kf.addJointGroup("arm_eef", kfs::KeyFrame::IS_JOINT_SPACE);
		kf.setJointSpaceVelocity(0.2);

		kf.setJointPosition("suc_finger_joint", 0);
		kf.setJointPosition("suc_fingertip_joint", 0);

		motion.push_back(kf);

		kfs::PlayMotionGoal goal;

		goal.motion_msg = motion.toMsg();
		goal.motion_name="reset_motion";
		goal.use_existing_motion=false;

		return goal;
	}

	nimbro_fsm::StateBase* Reset::nextState()
	{
		driver()->switchVacuum(false);
		driver()->switchLight(0);
		return 0;
	}

}
