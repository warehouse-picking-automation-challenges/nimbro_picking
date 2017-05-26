// APC objects
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef APC_OBJECTS_H
#define APC_OBJECTS_H

#include <string>
#include <vector>
#include <map>
#include <Eigen/Core>

namespace apc_objects
{

struct APCObject
{
	enum class HeuristicGrasp
	{
		Top,
		Center,
	};
	
	enum class PerceptionMethod
	{
		DenseCap,
		RBO,
	};

	APCObject();

	std::string imagePath() const;

	std::string name;
	std::vector<HeuristicGrasp> heuristicGrasps;

	PerceptionMethod shelfPerceptionMethod;
	PerceptionMethod totePerceptionMethod;

	bool topGraspHigh;

	int bonusPoints;
	Eigen::Vector3f bBox;
	float volume() const;
	float standingHeight;
	int difficulty;

	bool big;

	int erode;

	bool pick_forbidden;
	bool stow_forbidden;

	float suctionStrength;
	float airVelThreshold;

	float mass;

	bool stowGraspOriented;

	bool registration;
};

typedef std::map<std::string, APCObject> ObjectMap;

const ObjectMap& objects();

const float distRobot2Shelf = 1.7f;

}

#endif
