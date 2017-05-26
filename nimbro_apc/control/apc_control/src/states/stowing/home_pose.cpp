// Home state for APC
// Author: Christian Lenz <chrislenz@uni-bonn.de>

#include <ros/console.h>
#include <apc_control/control.h>
#include <apc_control/states/stowing/home_pose.h>

#include <nimbro_keyframe_server/motion.h>
#include <nimbro_keyframe_server/keyframe.h>

namespace apc_control
{

	void HomePose::enter()
	{
		MoveArm::enter();
	}

	nimbro_keyframe_server::PlayMotionGoal HomePose::computeGoal()
	{
		namespace kfs = nimbro_keyframe_server;

		WorkItem current_item = driver()->getCurrentItem();
		std::vector<float> boxCoordinates = apc_shelf_model::getBoxCoordinates(current_item.dest_row,current_item.dest_col);

		kfs::Motion motion;
		motion.setName("stow_home");

		// Turn finger straight
		{
			kfs::KeyFrame kf;

			kf.setLabel("finger straight");
			kf.addJointGroup("arm_eef", kfs::KeyFrame::IS_JOINT_SPACE);
			kf.setJointPosition("suc_fingertip_joint", 0.0);
			kf.setLinearVelocity(4.0);

			motion.push_back(kf);
		}

		// Retract finger
		{
			kfs::KeyFrame kf;

			kf.setLabel("finger retract");
			kf.addJointGroup("arm_eef", kfs::KeyFrame::IS_JOINT_SPACE);
			kf.setJointPosition("suc_finger_joint", 0.0);
			kf.setLinearVelocity(0.2);

			motion.push_back(kf);
		}

		// Retract arm
		{
			kfs::KeyFrame kf;

			kf.setLabel("[shelf] arm retract");
			kf.addJointGroup("arm", kfs::KeyFrame::IS_CARTESIAN);

			Eigen::Affine3d pose;
			pose = Eigen::Translation3d(
				1.0,
				0.5 * boxCoordinates[2] + 0.5 * boxCoordinates[3],
				apc_shelf_model::getBoxMaxZ(current_item.dest_row)
			);

			kf.setState("arm", pose);

			kf.setLinearVelocity(0.2);
			kf.setAngularVelocity(0.2);

			motion.push_back(kf);
		}

		nimbro_keyframe_server::PlayMotionGoal goal;

		goal.motion_msg = motion.toMsg();
		goal.motion_name="stow_home";
		goal.use_existing_motion=false;

		return goal;
	}

	nimbro_fsm::StateBase* HomePose::nextState()
	{
		return 0;
	}

}
