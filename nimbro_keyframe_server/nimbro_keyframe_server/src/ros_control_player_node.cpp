// Interface to ROS control (FollowJointTrajectory action)
// Author: Max Schwarz <max.schwarz@uni-bonn.de>
// Author: Sebastian Schüller <schuell1@cs.uni-bonn.de>

#include <ros_control_player/ros_control_player.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ros_control_player");

	ros::NodeHandle nh("~");

	nimbro_keyframe_server::ROSControlPlayer player;

	if(!player.init())
	{
		ROS_ERROR("Could not initialize player");
		return 1;
	}

	ros::spin();

	return 0;
}
