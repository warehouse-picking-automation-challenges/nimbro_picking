// Utilities
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

// NOTE(sebastian): This is a copy of sb_mission_control/utils.h.
// If changes / additions are wanted, maybe you should think about unifying
// both files somewhere.

#ifndef UTILS_H
#define UTILS_H

#include <ros/time.h>
#include <ros/init.h>
#include <ros/rate.h>

namespace utils
{

template<class Client>
bool waitForActionClient(Client* client, const ros::Duration& timeout = ros::Duration(5000))
{
	ros::Time start = ros::Time::now();
	ros::Rate rate(ros::Duration(0.1));

	while(ros::ok() && (ros::Time::now() - start) < timeout)
	{
		if(client->isServerConnected())
			return true;

		ros::spinOnce();

		rate.sleep();
	}

	return false;
}

}

#endif
