// Library to interpolate, collision-check and score motions
// Author: Sebastian Schüller <schuell1@cs.uni-bonn.de>

#ifndef APC_WORLD_H
#define APC_WORLD_H

#include <ros/node_handle.h>
#include <moveit/collision_detection/world.h>

namespace apc_world
{
struct APCWorld
{
	collision_detection::WorldPtr world;
	shapes::ShapePtr shelf;
	bool create();
	bool update(ros::NodeHandle nh = ros::NodeHandle());
	std::vector<std::pair<std::string, std::string>> getAllowedCollisions();
};
}

#endif
