// Library to interpolate, collision-check and score motions
// Author: Sebastian Schüller <schuell1@cs.uni-bonn.de>

#ifndef TRAJECTORY_SCORE_H
#define TRAJECTORY_SCORE_H

#include <ros/time.h>
#include <ros/node_handle.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection/world.h>

namespace nimbro_keyframe_server
{
	class MoveItPlayer;
	class Motion;
}


namespace trajectory_score
{

struct Score
{
	trajectory_msgs::JointTrajectory trajectory;
	bool inCollision;
	double minDistance;
	ros::Duration estimatedDuration;
};

class TrajectoryScorer
{
public:
	// NOTE (sebastian): Caller is responsible to keep the RobotState
	// pointer and to set it correctly before each call to scoreMotion()!
	explicit TrajectoryScorer(
		const robot_state::RobotStatePtr& state,
		const collision_detection::WorldPtr& world,
		const robot_model::RobotModelPtr& model,
		ros::Duration period = ros::Duration(1.0f / 20.0f)
	);
	explicit TrajectoryScorer(
		const robot_state::RobotStatePtr& state,
		const robot_model::RobotModelPtr& model,
		ros::Duration period = ros::Duration(1.0f / 20.0f)
	);

	~TrajectoryScorer();


	void setAllowedCollisions(
		std::vector<std::pair<std::string, std::string> > pairs);
	void setPadding(float padding);
	void setCollisionCheckingEnabled(bool enabled);
	Score scoreMotion(
		boost::shared_ptr<nimbro_keyframe_server::Motion> motion,
		std::size_t subsampling = 1);
private:
	robot_model::RobotModelPtr m_model;
	robot_state::RobotStatePtr m_state;
	collision_detection::WorldPtr m_world;
	std::vector<std::pair<std::string, std::string> > m_allowedCollisions;
	float m_padding;
	bool m_collisionCheckingEnabled;

	ros::Duration m_period;
	boost::shared_ptr<nimbro_keyframe_server::MoveItPlayer> m_player;
};
}

#endif
