// Represents a MoveIt group (e.g. left arm)
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "body_part.h"

#include <config_server/parameter.h>

namespace apc_motion
{

BodyPart::BodyPart(robotcontrol::RobotModel* robotModel,
	const robot_model::RobotModelPtr& moveItModel,
	const robot_state::RobotStatePtr& moveItState,
	const robot_model::JointModelGroup* moveItGroup)
 : m_model(robotModel)
 , m_moveItModel(moveItModel)
 , m_moveItState(moveItState)
 , m_moveItGroup(moveItGroup)
 , m_param_alpha("bodyparts/" + moveItGroup->getName() + "/alpha", 0.0, 0.001, 1.0, 1.0)
 , m_alpha(1.0)
{
	// Joint mapping
	unsigned int dof = m_moveItGroup->getVariableCount();
	const std::vector<std::string>& names = m_moveItGroup->getVariableNames();
	m_moveItVariableIndices = m_moveItGroup->getVariableIndexList();

	m_joints.resize(dof);

	for(unsigned int i = 0; i < dof; ++i)
	{
		robotcontrol::Joint::Ptr joint = m_model->getJoint(names[i]);

		if(!joint)
		{
			std::stringstream ss;
			ss << "Could not find joint '" << names[i] << "' in robotcontrol model";
			throw std::logic_error(ss.str().c_str());
		}

		ROS_DEBUG("BodyPart '%s': %s -> %d", m_moveItGroup->getName().c_str(),
			names[i].c_str(), m_moveItVariableIndices[i]
		);
		m_joints[i] = joint;
	}

	// Initial pose
	for(unsigned int i = 0; i < dof; ++i)
		m_moveItState->setVariablePosition(m_moveItVariableIndices[i], 0.0);

	config_server::Parameter<std::string> initPose("/momaro/init_pose", "init");
	std::string launchInitPose;
	ros::param::param<std::string>("/init_pose", launchInitPose, initPose());

	m_moveItState->setToDefaultValues(m_moveItGroup, launchInitPose);
}

BodyPart::~BodyPart()
{
}

void BodyPart::step()
{
	m_alpha = 0.94 * m_alpha + 0.06 * m_param_alpha();

	m_moveItState->copyJointGroupPositions(m_moveItGroup, m_moveItPositions);
	
	{
		const std::vector<int> &il = m_moveItGroup->getVariableIndexList();
		m_moveItVelocities.resize(il.size());
// 		if (m_moveItGroup->isContiguousWithinState())
// 			memcpy(m_moveItVelocities, m_moveItState->getVariableVelocities() + il[0], m_moveItGroup->getVariableCount() * sizeof(double));
// 		else
			for (std::size_t i = 0 ; i < il.size() ; ++i)
			m_moveItVelocities[i] = m_moveItState->getVariableVelocities()[il[i]];
	}
	
	if(m_moveItPositions.size() != m_joints.size())
	{
		ROS_FATAL("Variable counts mismatch: moveit: %d, we: %d", (int)m_moveItPositions.size(), (int)m_joints.size());
		throw std::runtime_error("moveit error");
	}

	for(unsigned int i = 0; i < m_joints.size(); ++i)
	{
		const robotcontrol::Joint::Ptr& joint = m_joints[i];

		if(!std::isfinite(m_moveItPositions[i]))
		{
			ROS_WARN_THROTTLE(1.0, "NaN on moveit joint '%s'", joint->name.c_str());
		}
		else if(m_alpha > 0.05)
		{
			double pos = m_alpha * m_moveItPositions[i] + (1.0 - m_alpha) * joint->cmd.pos;
			double vel = m_alpha * m_moveItVelocities[i] + (1.0 - m_alpha) * joint->cmd.vel;

			joint->cmd.pos = pos;
			joint->cmd.vel = vel;
// 			joint->cmd.setFromPos(m_model->timerDuration(), pos); // DANGEROUS!
		}
	}
}

}
