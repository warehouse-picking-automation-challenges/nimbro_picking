// Scan pose state for APC
// Author: Christian Lenz <chrislenz@uni-bonn.de>

#include <ros/console.h>
#include <apc_control/control.h>
#include <apc_control/states/picking/box_pose.h>
#include <apc_control/states/picking/pregrasp.h>

namespace apc_control
{

	nimbro_keyframe_server::PlayMotionGoal BoxPose::computeGoal()
	{
		nimbro_keyframe_server::PlayMotionGoal goal;

		WorkItem current_item = driver()->getCurrentItem();

		std::string motion_name="grasp_";
		motion_name += current_item.location;

		goal.motion_name=motion_name;
		goal.use_existing_motion=true;

		return goal;
	}

	nimbro_fsm::StateBase* BoxPose::nextState()
	{
		return new apc_control::Pregrasp();
	}


}