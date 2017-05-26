// Scan pose state for APC
// Author: Christian Lenz <chrislenz@uni-bonn.de>

#include <ros/console.h>
#include <apc_control/control.h>
#include <apc_control/states/stowing/stow_pose.h>
#include <apc_control/states/stowing/stow_item.h>
#include <apc_control/states/stowing/home_pose.h>

#include <nimbro_keyframe_server/motion.h>
#include <Eigen/Geometry>

namespace apc_control
{


	nimbro_keyframe_server::PlayMotionGoal StowPose::computeGoal()
	{
		nimbro_keyframe_server::PlayMotionGoal goal;

		ROS_INFO("stow pose msg:\n%s", driver()->getStowBox()->serializeToYAML().c_str());

		goal.motion_msg = driver()->getStowBox()->toMsg();
		goal.motion_name="stow_box";
		goal.use_existing_motion=false;

		return goal;
	}

	nimbro_fsm::StateBase* StowPose::nextState()
	{
		if(driver()->somethingOnTip())
		{
			driver()->setItemPlaced(true);
			return new StowItem();
		}
		else
		{
			driver()->setItemPlaced(false);
			driver()->switchVacuum(false);
			return new HomePose();
		}
	}


}
