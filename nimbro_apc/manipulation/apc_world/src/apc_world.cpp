// Library to interpolate, collision-check and score motions
// Author: Sebastian Schüller <schuell1@cs.uni-bonn.de>

#include <apc_world/apc_world.h>
#include <geometric_shapes/mesh_operations.h>

#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>

#include <apc_objects/apc_objects.h>

#include <ros/package.h>

namespace apc_world
{


bool APCWorld::create()
{
	world.reset(new collision_detection::World());

	Eigen::Affine3d defaultShelfPose = Eigen::Affine3d::Identity();
	defaultShelfPose.translation().x() = apc_objects::distRobot2Shelf;

	shelf.reset(
		shapes::createMeshFromResource(
			"file://" + ros::package::getPath("apc_model") +
			"/meshes/shelf_collision_rotated" + ".stl")
	);

	world->addToObject("shelf" , shelf, defaultShelfPose);
	return true;
}

bool APCWorld::update(ros::NodeHandle nh)
{
	tf::TransformListener tf{nh};
	tf::StampedTransform shelfTransform;
	try
	{
		tf.lookupTransform(
			"/world", "/perception_shelf",
			ros::Time(0), shelfTransform);
	}
	catch(tf::TransformException e)
	{
		ROS_ERROR("Couldn't get current shelf transform!");
		return false;
	}

	Eigen::Affine3d shelfPose;
	tf::transformTFToEigen(shelfTransform, shelfPose);

	world->moveShapeInObject("shelf", shelf, shelfPose);
	return true;
}

std::vector<std::pair<std::string, std::string>>
APCWorld::getAllowedCollisions()
{
	std::vector<std::pair<std::string, std::string>> pairs = {
		{ "table_link",     "shelf" },
		{ "base_link",      "shelf" },
		{ "forearm_link",   "shelf" },
		{ "shoulder_link",  "shelf" },
		{ "upper_arm_link", "shelf" }
	};
	return pairs;
}
}
