// Pregrasp state for APC
// Author: Christian Lenz <chrislenz@uni-bonn.de>

#include <ros/console.h>
#include <apc_control/control.h>
#include <apc_control/states/picking/pregrasp.h>
#include <apc_control/states/picking/grasp.h>

#include <apc_control/apc_database.h>

#include <nimbro_keyframe_server/motion.h>

#include <geometry_msgs/PoseStamped.h>

#include <eigen_conversions/eigen_msg.h>


namespace apc_control
{


	nimbro_keyframe_server::PlayMotionGoal Pregrasp::computeGoal()
	{

		nimbro_keyframe_server::PlayMotionGoal goal;

		goal.motion_msg = driver()->getPregrasp()->toMsg();
		goal.motion_name="pregrasp";
		goal.use_existing_motion=false;

		ROS_INFO("requesting pregrasp motion:\n%s", driver()->getPregrasp()->serializeToYAML().c_str());

		return goal;
	}

	nimbro_fsm::StateBase* Pregrasp::nextState()
	{
		return new apc_control::Grasp();
	}
}
