// Test TrajectoryScorer in Simulation
// Author: Sebastian Schüller <schuell1@cs.uni-bonn.de>

#include <apc_world/apc_world.h>
#include <trajectory_score/trajectory_score.h>

#include <ros/init.h>
#include <ros/console.h>

#include <ros/package.h>
#include <geometric_shapes/mesh_operations.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

#include <nimbro_keyframe_server/motion.h>
#include <nimbro_keyframe_server/keyframe_server.h>

#include <apc_objects/apc_objects.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "trajectory_score_sim_test");
	ros::NodeHandle nh;

	auto loader = robot_model_loader::RobotModelLoader("robot_description");
	auto model = loader.getModel();
	using namespace nimbro_keyframe_server;
	const std::string motionName = "self_collision";
	KeyFrameServer s;
	using namespace trajectory_score;
	using namespace apc_world;

	ROS_INFO("\nRunning lib version");
	{
		APCWorld apcWorld = {};
		if(!apcWorld.create())
		{
			ROS_ERROR("Failed to create world!");
			return 1;
		}
		boost::shared_ptr<robot_state::RobotState> state(
			new robot_state::RobotState(model));
		state->setToDefaultValues();
		for(auto& group : model->getJointModelGroups())
		{
			state->setToDefaultValues(group, "init");
		}

		collision_detection::WorldPtr w(new collision_detection::World());

		TrajectoryScorer scorer{state, w/*apcWorld.world*/, model};
		scorer.setPadding(0.02);
		//scorer.setAllowedCollisions(apcWorld.getAllowedCollisions());
		Motion::Ptr m = s.getMotion(motionName);
		scorer.scoreMotion(m, 3);
	}
#if 0
	ROS_INFO("\nRunning manual version");
	// Use collision shelf
	{
		collision_detection::WorldPtr world{
			new collision_detection::World{}
		};
		auto path = "file://" + ros::package::getPath("apc_model") +
			"/meshes/shelf_collision_rotated.stl";
		shapes::ShapePtr shape { shapes::createMeshFromResource(path) };
		Eigen::Affine3d pose = Eigen::Affine3d::Identity();
		pose.translation().x() = apc_objects::distRobot2Shelf;
		world->addToObject("shelf", shape, pose);

		boost::shared_ptr<robot_state::RobotState> state{
			new robot_state::RobotState(model)};
		state->setToDefaultValues();
		for(auto& group : model->getJointModelGroups())
		{
			state->setToDefaultValues(group, "init");
		}
		TrajectoryScorer scorer{state, world, model};
		Motion::Ptr m = s.getMotion(motionName);
		auto r = scorer.scoreMotion(m, 2);
		ROS_INFO("Estimated Time: %f", r.estimatedDuration.toSec());
	}
	ROS_INFO("\nRunning parts version");
	// Use collision shelf parts
	{
		collision_detection::WorldPtr world{
			new collision_detection::World{}
		};

		Eigen::Affine3d pose = Eigen::Affine3d::Identity();
		pose.translation().x() = apc_objects::distRobot2Shelf;

		std::vector<shapes::ShapePtr> parts{22};
		for(int i = 0; i < 22; ++i)
		{
#if 0
			std::stringstream ss;
			ss << "uiae";
			ROS_INFO("%s %p", ss.str().c_str(), &ss);
#endif
			parts[i].reset(
				shapes::createMeshFromResource(
					"file://"+ros::package::getPath("apc_model")+
					"/meshes/shelf_collision_part_"+
					std::to_string(i)+".stl")
			);
			world->addToObject("shelf", parts[i], pose);
		}
		boost::shared_ptr<robot_state::RobotState> state(
			new robot_state::RobotState(model));
		state->setToDefaultValues();
		for(auto& group : model->getJointModelGroups())
		{
			state->setToDefaultValues(group, "init");
		}
		TrajectoryScorer scorer{state, world, model};
		Motion::Ptr m = s.getMotion(motionName);
		scorer.scoreMotion(m);
	}
#endif
	return 0;

}
