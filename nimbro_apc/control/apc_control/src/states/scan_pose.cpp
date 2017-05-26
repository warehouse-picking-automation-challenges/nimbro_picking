// Scan pose state for APC
// Author: Christian Lenz <chrislenz@uni-bonn.de>

#include <ros/console.h>
#include <apc_control/control.h>
#include <apc_control/states/scan_pose.h>
#include <apc_control/states/perception.h>

namespace apc_control
{

nimbro_keyframe_server::PlayMotionGoal ScanPose::computeGoal()
{
	nimbro_keyframe_server::PlayMotionGoal goal;

	WorkItem current_item = driver()->getCurrentItem();

	std::string motion_name="capture_";

	if(driver()->getIsPicking())
		motion_name += current_item.location;
	else
		motion_name += "tote_angle";

	goal.motion_name=motion_name;
	goal.use_existing_motion=true;

	return goal;
}

nimbro_fsm::StateBase* ScanPose::nextState()
{
	return new PerceptionDelay();
}

}
