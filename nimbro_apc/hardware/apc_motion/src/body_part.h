// Represents a MoveIt group (e.g. left arm)
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef BODY_PART_H
#define BODY_PART_H

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <robotcontrol/model/robotmodel.h>

namespace apc_motion
{

class BodyPart
{
public:
	typedef boost::shared_ptr<BodyPart> Ptr;

	BodyPart(robotcontrol::RobotModel* robotModel,
		const robot_model::RobotModelPtr& moveItModel,
		const robot_state::RobotStatePtr& moveItState,
		const robot_model::JointModelGroup* moveItGroup
	);

	~BodyPart();

	void step();
private:
	robotcontrol::RobotModel* m_model;
	robot_model::RobotModelPtr m_moveItModel;
	robot_state::RobotStatePtr m_moveItState;

	const robot_model::JointModelGroup* m_moveItGroup;

	std::vector<robotcontrol::Joint::Ptr> m_joints;
	std::vector<int> m_moveItVariableIndices;

	std::vector<double> m_moveItPositions;
	std::vector<double> m_moveItVelocities;

	config_server::Parameter<float> m_param_alpha;
	double m_alpha;
};

}

#endif
