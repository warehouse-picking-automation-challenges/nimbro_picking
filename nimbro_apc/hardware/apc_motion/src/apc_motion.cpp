// APC arm motion generator
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "apc_motion.h"

#include <robotcontrol/model/robotmodel.h>
#include <robotcontrol/robotcontrol.h>
#include <robotcontrol/model/singlesupportmodel.h>

#include <pluginlib/class_list_macros.h>

#include <eigen_conversions/eigen_msg.h>

#include <moveit/robot_model_loader/robot_model_loader.h>

#include <Eigen/Geometry>

#include <rbdl/treestream.h>

#include <visualization_msgs/Marker.h>

namespace apc_motion
{

APCMotion::APCMotion()
 : m_armControl_lin_vel("/momaro/arm_control/lin_vel", 0.01, 0.01, 2.0, 0.2)
 , m_armControl_rot_vel("/momaro/arm_control/rot_vel", 0.01, 0.01, 2.0, 0.3)
 , m_model(0)
{
}

APCMotion::~APCMotion()
{
}

bool APCMotion::init(robotcontrol::RobotModel* model)
{
	if(!robotcontrol::MotionModule::init(model))
		return false;

	// MoveIt init
	m_moveitLoader = boost::make_shared<robot_model_loader::RobotModelLoader>(
		"robot_description"
	);
	m_moveitModel = m_moveitLoader->getModel();
	ROS_DEBUG("Model frame: %s", m_moveitModel->getModelFrame().c_str());

	m_moveitState.reset(new robot_state::RobotState(m_moveitModel));
	m_moveitState->setToDefaultValues();

	// Init groups
	for(const robot_model::JointModelGroup* group : m_moveitModel->getJointModelGroups())
	{
		m_parts.emplace_back(new BodyPart(
			model,
			m_moveitModel,
			m_moveitState,
			group
		));
	}

	// MoveIt player
	m_player.reset(new nimbro_keyframe_server::MoveItPlayer(
		m_moveitModel,
		m_moveitState,
		ros::Duration(model->timerDuration())
	));

	m_keyframeServer.setPlayCallback(boost::bind(&nimbro_keyframe_server::MoveItPlayer::playMotion, m_player.get(), _1));
	m_player->setStartCallback(boost::bind(&APCMotion::handleStartKeyframe, this, _1));
	m_player->setEndCallback(boost::bind(&APCMotion::handleEndKeyframe, this, _1));

	m_pub_plot = ros::NodeHandle().advertise<plot_msgs::Plot>("/plot", 10);
	plot_msgs::PlotPoint p;
	p.name = "/upper_body/jointSpaceLimiting";
	m_plot.points.push_back(p);

	m_model = model;
	m_state_unsafe_connection = model->registerState("unsafe_connection");

	return true;
}

bool APCMotion::isTriggered()
{
	return true;
}

void APCMotion::step()
{
	if(!m_model)
		return;

	if(m_model->state() == m_state_unsafe_connection)
	{
		std::vector<std::string> jointNames;
		std::vector<double> jointPositions;
		for(size_t i = 0; i < m_model->numJoints(); ++i)
		{
			auto joint = (*m_model)[i];
			jointNames.push_back(joint->name);
			jointPositions.push_back(joint->feedback.pos);
		}
		try
		{
			m_moveitState->setVariablePositions(jointNames, jointPositions);
		}
		catch(std::exception& e)
		{
			ROS_WARN("[UpperBody] Failed to set servos in unsafe state with Message : %s", e.what());
		}
	}
	else
	m_player->step();

	m_plot.header.stamp = ros::Time::now();
	m_plot.points[0].value = m_player->limitingJointSpaceVelocity() ? 1.0 : 0.0;
	m_pub_plot.publish(m_plot);

	for(BodyPart::Ptr& part : m_parts)
		part->step();
}

bool APCMotion::playMotion(const nimbro_keyframe_server::Motion::Ptr& motion)
{
	return m_player->playMotion(motion);
}

void APCMotion::playMotionGoal()
{

}

std::vector<std::string> APCMotion::getJointSpaceJoints(const nimbro_keyframe_server::KeyFrame& frame)
{
	std::vector<std::string> joints;

	for(auto& group : frame.jointGroups())
	{
		if(group.second.interpolationSpace() != nimbro_keyframe_server::KeyFrame::IS_JOINT_SPACE)
			continue;

		auto jointGroup = m_moveitModel->getJointModelGroup(group.first);

		if(!jointGroup)
			continue;

		for(auto& joint : jointGroup->getJointModels())
		{
			if(joint->getType() != moveit::core::JointModel::REVOLUTE)
				continue;

			joints.push_back(joint->getName());
		}
	}

	return joints;
}

void APCMotion::handleStartKeyframe(const nimbro_keyframe_server::KeyFrame& frame)
{
}

void APCMotion::handleEndKeyframe(const nimbro_keyframe_server::KeyFrame& frame)
{
}

void APCMotion::handleEmergencyStop()
{
	m_player->stopMotion();
}

}

PLUGINLIB_EXPORT_CLASS(apc_motion::APCMotion, robotcontrol::MotionModule)
