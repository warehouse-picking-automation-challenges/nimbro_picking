// Library to interpolate, collision-check and score motions
// Author: Sebastian Schüller <schuell1@cs.uni-bonn.de>

#include <trajectory_score/trajectory_score.h>

#include <moveit/robot_state/robot_state.h>

#include <nimbro_keyframe_server/moveit_player.h>

#include <geometric_shapes/mesh_operations.h>

#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>

#include <ros/package.h>
#include <fstream>

static void coredump_now()
{
	if(!fork()) {
		std::abort();
	}
}

namespace trajectory_score
{

TrajectoryScorer::TrajectoryScorer(
	const robot_state::RobotStatePtr& state,
	const collision_detection::WorldPtr& world,
	const robot_model::RobotModelPtr& model,
	ros::Duration period)
 : m_model(model)
 , m_state(state)
 , m_world(world)
 , m_padding(0.0f)
 , m_period(period)
 , m_collisionCheckingEnabled(true)
{
	using namespace nimbro_keyframe_server;
	m_player.reset(new MoveItPlayer(m_model, state, period));
	m_player->useInternalCollisionChecking(false);
}

TrajectoryScorer::TrajectoryScorer(
	const robot_state::RobotStatePtr& state,
	const robot_model::RobotModelPtr& model,
	ros::Duration period)
 : m_model(model)
 , m_state(state)
 , m_padding(0.0f)
 , m_period(period)
 , m_collisionCheckingEnabled(true)
{
	m_world.reset(
		new collision_detection::World());
	using namespace nimbro_keyframe_server;
	m_player.reset(new MoveItPlayer(m_model, state, period));
	m_player->useInternalCollisionChecking(false);
}

TrajectoryScorer::~TrajectoryScorer()
{
}

static trajectory_msgs::JointTrajectoryPoint trajecoryPointFromState(
	const std::vector<int>& jointIndices,
	const robot_state::RobotStatePtr& state)
{
	trajectory_msgs::JointTrajectoryPoint point = {};
	point.positions.resize(jointIndices.size());
	point.velocities.resize(jointIndices.size());
	for(size_t i = 0; i < jointIndices.size(); ++i)
	{
		auto index = jointIndices[i];
		point.positions[i] = state->getVariablePosition(index);
		point.velocities[i] = state->getVariableVelocity(index);
	}

	return point;
}

void TrajectoryScorer::setAllowedCollisions(
	std::vector<std::pair<std::string, std::string> > pairs
)
{
	m_allowedCollisions = pairs;
}

void TrajectoryScorer::setPadding(float padding)
{
	m_padding = padding;
}

void TrajectoryScorer::setCollisionCheckingEnabled(bool enabled)
{
	m_collisionCheckingEnabled = enabled;
}

Score TrajectoryScorer::scoreMotion(
	boost::shared_ptr<nimbro_keyframe_server::Motion> motion,
	std::size_t subsampling)
{
	auto debugTimeStart = ros::Time::now();
	Score score = {};
	score.minDistance = std::numeric_limits<double>::infinity();

	std::vector<int> jointIndices; // map involved joints to moveit joints
	std::set<std::string> cartesian_groups;

	for(auto& kf : *motion)
	{
		for(const auto& group : kf.jointGroups())
		{
			auto jointGroup = m_model->getJointModelGroup(group.first);
			if(!jointGroup)
			{
				std::stringstream ss;
				ss << "Unkown Joint Group '" << group.first << "'"
				   << " found in Motion '" <<  motion->name() << "'";
				throw std::logic_error(ss.str());
			}

			if(group.second.interpolationSpace()==nimbro_keyframe_server::KeyFrame::IS_CARTESIAN)
				cartesian_groups.insert(group.first);

			auto& names = jointGroup->getJointModelNames();
			for(auto& name : names)
			{
				auto* model = m_model->getJointModel(name);
				auto type = model->getType();
				if(type == robot_model::JointModel::FIXED)
					continue;

				auto it = std::find(score.trajectory.joint_names.begin(), score.trajectory.joint_names.end(), name);
				if(it != score.trajectory.joint_names.end())
					continue;

				score.trajectory.joint_names.push_back(name);
				auto index = model->getFirstVariableIndex();
				jointIndices.push_back(index);
			}
		}
	}



	std::vector<robot_state::RobotState> states;

	// Interpolate keyframes
	auto interpolationStart = ros::Time::now();
	auto timeFromStart = ros::Duration(0);
	m_player->playMotion(motion);
	ROS_INFO("Motion in trajectory score: %s", motion->serializeToYAML().c_str());


	std::string path = "/tmp/state_" + motion->name() + ".txt";
	std::ofstream out(path.c_str());

	out << "time ";
	for(auto name : score.trajectory.joint_names)
		out << "pos_" << name << " vel_" << name << " ";

	for(auto jg: cartesian_groups){
		out << "eef_pos_" << jg << "_x ";
		out << "eef_pos_" << jg << "_y ";
		out << "eef_pos_" << jg << "_z ";
		out << "eef_pos_" << jg << "_rot ";
		out << "eef_vel_" << jg << "_x ";
		out << "eef_vel_" << jg << "_y ";
		out << "eef_vel_" << jg << "_z ";
		out << "eef_vel_" << jg << "_rot ";
		out << "eef_acc_" << jg << "_x ";
		out << "eef_acc_" << jg << "_y ";
		out << "eef_acc_" << jg << "_z ";
		out << "eef_acc_" << jg << "_rot ";

	}
	out << "\n";



	do
	{
		auto trajectoryPoint =
			trajecoryPointFromState(jointIndices, m_state);
		trajectoryPoint.time_from_start = timeFromStart;
		score.trajectory.points.push_back(std::move(trajectoryPoint));

		out << timeFromStart.toSec() << " ";
		for(size_t i = 0; i < jointIndices.size(); ++i)
		{
			auto index = jointIndices[i];
			out << m_state->getVariablePosition(index) << " " << m_state->getVariableVelocity(index) << " ";
		}

		for(auto jg: cartesian_groups)
		{
			Eigen::Vector4d info = m_player->getEndEffectorPosition(jg);
			for(unsigned int i=0;i<4;i++)
				out << info[i] << " ";
			info=m_player->getEndEffectorVelocity(jg);
			for(unsigned int i=0;i<4;i++)
				out << info[i] << " ";
			info=m_player->getEndEffectorAcceleration(jg);
			for(unsigned int i=0;i<4;i++)
				out << info[i] << " ";
		}
		out << "\n";

		timeFromStart += m_period;
		states.push_back(*m_state);

		if((ros::Time::now() - interpolationStart) > ros::Duration(3.0f))
		{
			ROS_ERROR("--------------------------------------------------------------------------------");
			ROS_ERROR("------- Interpolation took longer than a second!");
			ROS_ERROR("----- This will be reported as a collision for now!");
			ROS_ERROR("----------- Interpolation had %zu frames", states.size());
			ROS_ERROR("--------------------------------------------------------------------------------");

			m_player->debugDump();

			coredump_now();

			score.inCollision = true;
			score.trajectory = trajectory_msgs::JointTrajectory();
			return score;
		}
	} while(!m_player->step());
	auto trajectoryPoint = trajecoryPointFromState(jointIndices, m_state);
	trajectoryPoint.time_from_start = timeFromStart;
	score.trajectory.points.push_back(std::move(trajectoryPoint));
	score.estimatedDuration = timeFromStart;

	if(m_collisionCheckingEnabled)
	{
		auto start = ros::Time::now();

		collision_detection::CollisionRequest req = {};
		req.contacts = true;
		req.max_contacts = 1000;
		req.distance = true;
		std::vector<collision_detection::CollisionResult> results{states.size()};

		#pragma omp parallel
		{
			collision_detection::WorldPtr w{
				new collision_detection::World{*m_world}
			};
			planning_scene::PlanningScene s{m_model, w};
			auto& allowedCollisions =
				s.getAllowedCollisionMatrixNonConst();
			for(const auto& pair : m_allowedCollisions)
				allowedCollisions.setEntry(
					pair.first, pair.second, true);
			auto& collRobot = s.getCollisionRobotNonConst();
			collRobot->setPadding(m_padding);
			#pragma omp for
			for(size_t i = 0; i < states.size(); i += subsampling)
			{
				auto& state = states[i];
				auto& res = results[i];

				s.setCurrentState(state);
				s.checkCollision(req, res);
			}

		}
		ROS_INFO("Collision checking took: %f s", (ros::Time::now() - start).toSec());


		for(auto& r : results)
		{
			for(const auto& c : r.contacts)
				ROS_ERROR("Collision between : '%s' | '%s'", c.first.first.c_str(), c.first.second.c_str());
			if(r.collision)
			{
				score.inCollision = true;
			}
			if(r.distance < score.minDistance)
				score.minDistance = r.distance;
		}
		ROS_INFO("Collision State: %s", score.inCollision ? "true" : "false");
	}
	else
	{
		ROS_INFO("No collision checking performed.");
		score.inCollision = false;
	}

	return score;
}

}
