// Motion player for MoveIt
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <nimbro_keyframe_server/moveit_player.h>

#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <ros/console.h>

#include <Eigen/Geometry>
#include <nimbro_collision_detection/CollisionDetection.h>

namespace nimbro_keyframe_server
{

const std::string UNRESOLVED_COLLISION_DETECTION_SRV = "collision_detection/collisionDetection";

template<class T>
T robustAcos(T x)
{
	return std::acos(std::max<T>(-1.0, std::min<T>(1.0, x)));
}


MoveItPlayer::MoveItPlayer(const robot_model::RobotModelPtr& model,
	const robot_state::RobotStatePtr& state,
	ros::Duration dt)
 : m_model(model)
 , m_state(state)
 , m_dt(dt)

 , m_maxLinearAcceleration("/moveit_player/max_acceleration/linear", 0.0, 0.01, 1.0, 0.3)
 , m_maxAngularAcceleration("/moveit_player/max_acceleration/angular", 0.0, 0.01, 1.0, 0.3)
 , m_maxJointSpaceAcceleration("/moveit_player/max_acceleration/joint_space", 0.0, 0.01, 20.0, 0.9)

 , m_maxLinearVelocity("/moveit_player/max_velocity/linear",0.0,0.01,100.0,30.0)
 , m_maxAngularVelocity("/moveit_player/max_velocity/angular",0.0,0.01,100.0,30.0)
 , m_maxJointSpaceVelocity("/moveit_player/max_velocity/joint_space",0.0,0.01,100.0,30.0)
 , m_maxFinalJointSpaceVelocity("/moveit_player/max_final_joint_velocity_degree", 0.0, 0.1, 100.0, 1.0)

 , m_maxJerk("/moveit_player/max_jerk", 0.0, 0.01, 200.0, 0.2)
 , m_collisionDetection("/moveit_player/collision_detection/enabled", false)
 , m_skipStep(false)
 , m_interrupt(false)
 , m_limitingJointSpaceVelocity(false)
 , m_inCollision(false)
 , m_useCollisionChecking(true)
{
	// Remember all joint Groups
	const std::vector<moveit::core::JointModelGroup*>& modelGroups = m_model->getJointModelGroups();
	std::vector<moveit::core::JointModelGroup*>::const_iterator group = modelGroups.begin();

	ros::NodeHandle nh;

	for (; group != modelGroups.end(); ++group)
	{
		std::string groupName = (*group)->getName();
		m_groups[groupName] = Group();
		m_groups[groupName].jointGroup = *group;
		m_groups[groupName].interpolationSpace = KeyFrame::IS_NONE;
		std::string rotspace_param = "/robot_description_kinematics/" + groupName + "/moveit_player_rotspace";
		std::string rotspace;
		nh.param(rotspace_param,rotspace, std::string("full"));

		if(rotspace == "without_roll")
			m_groups[groupName].rotationSpace = Group::ROTSPACE_WITHOUT_ROLL;
		else if(rotspace == "without_pitch")
			m_groups[groupName].rotationSpace = Group::ROTSPACE_WITHOUT_PITCH;
		else if(rotspace == "without_yaw")
			m_groups[groupName].rotationSpace = Group::ROTSPACE_WITHOUT_YAW;
		else
			m_groups[groupName].rotationSpace = Group::ROTSPACE_FULL;
	}

	// Create a copy of the state for us to play in with IK
	m_ikState.reset(new robot_state::RobotState(*m_state));


	// Trajectory server for FollowJointTrajectory action
	// TODO: Can we move this somewhere else?
	ros::NodeHandle n("~");

	m_trajectoryServer.reset(
		new TrajectoryServer(n, "moveit_player/followJointTrajectory",
			boost::bind(&MoveItPlayer::trajectoryHandleGoal, this, _1),
			boost::bind(&MoveItPlayer::trajectoryCancelGoal, this, _1),
			false
		)
	);
	m_trajectoryServer->start();

	m_firstExec = true;

	m_unmodifiedRobotState.reset(new robot_state::RobotState(*m_state));

	m_collisionDetectionSrv = n.serviceClient<nimbro_collision_detection::CollisionDetection>(n.resolveName(UNRESOLVED_COLLISION_DETECTION_SRV));
}

MoveItPlayer::~MoveItPlayer()
{
}

bool MoveItPlayer::playMotion(const Motion::Ptr& motion)
{
	m_motion = motion;
	m_keyframeIndex = 0;

	std::string name = motion->name();

	m_skipStep =(name.substr(0, 4)=="skip")?true:false;
	m_interrupt = (name=="stop_motion")?true:false;

	setupReflexxes();

	m_inCollision = false;

	return true;
}

void MoveItPlayer::stopMotion()
{
	m_motion.reset();
}

/**
 * returns: the velocity the system is able to reach if it accelerates at the maximal acceleration from the current postion to the desired position
 */
//FIXME use only linear acceleration limit
static double calcMaxVel(double currentPosition,
	double desiredPosition,
	double currentVelocity,
	double maxAcceleration)
{
	double diff = desiredPosition-currentPosition;

	if(diff == 0.0)
		return 0.0;

	if(diff < 0.0)
	{
			maxAcceleration *= -1.0;
	}

	double head = - currentVelocity/maxAcceleration;
	double tail = std::pow(head, 2.0) + 2 * (diff/maxAcceleration);
	if(tail < 0.0) // should never happen
	{
		ROS_ERROR_STREAM("tail < 0.0");
	}
	tail = std::sqrt(tail);

	double t1 = head - tail;
	double t2 = head + tail;

	// select the correct t
	double t = std::max(t1, t2);

	// should never happen
	if((t1>=0 && t2>=0) || (t1<=0 && t2<=0))
	{
		ROS_ERROR_STREAM("(t1>=0 && t2>=0) || (t1<=0 && t2<=0)");
	}

	double maxVel = currentVelocity + maxAcceleration * t;

	return maxVel;
}

Eigen::Vector3d MoveItPlayer::calculateTargetVelocity(std::string groupName,
	Eigen::Vector3d currentPosition,
	Eigen::Vector3d desiredPosition,
	Eigen::Vector4d currentVelocity,
	double linearVelocity)
{
	// the desired velocity is the maximal possible velocity in the direction which points from the previous position to the next position
	Eigen::Vector3d targetVelocity;
	targetVelocity.setZero();
	if(m_keyframeIndex < (m_motion->size()-1)) // make sure that this keyframe is not the last in the motion
	{
			Eigen::Vector3d lastPosition;
			// if this keyframe is the first of the motion the last position is the current position
			if(m_keyframeIndex == 0)
			{
				lastPosition = currentPosition;
			}
			else
			{
				lastPosition = (&(*m_motion)[m_keyframeIndex-1])->state(groupName).translation();
			}

			Eigen::Vector3d nextPosition = (&(*m_motion)[m_keyframeIndex+1])->state(groupName).translation();
			Eigen::Vector3d velocityDirection = nextPosition - lastPosition;

			// check if this results to zero
			if(!velocityDirection.isZero())
			{
				velocityDirection.normalize();

				targetVelocity = velocityDirection * linearVelocity;
			}
	}

	if(!targetVelocity.isZero())
	{
		// the maximal value for alpha is 1.0, this ensures that we do not try to reach a target velocity which is higher than the desired velocity
		double alpha = 1.0;
		double maxAlpha = std::numeric_limits<double>::lowest();
		Eigen::Vector3d maxVel;
		maxVel.setZero();
		// The proportion of the desired velocity needs to be maintained, otherwise the direction whould not be correct
		for (unsigned int j = 0; j < 3; ++j)
		{
			maxVel(j) = calcMaxVel(currentPosition(j),
								   desiredPosition(j),
								   currentVelocity(j),
								   m_maxLinearAcceleration()
  								);

			double tmpAlpha = maxVel(j)/targetVelocity(j);

			if(tmpAlpha<alpha && tmpAlpha >= 0.0)
			{
					alpha = tmpAlpha;
			}
			if(tmpAlpha>maxAlpha)
			{
					maxAlpha = tmpAlpha;
			}
		}

		if(maxAlpha < 0.0)
		{
			alpha = 0.0;
		}

		// we cannot set the maximal reachable velocity as target velocity as the velocity profile of the q spline prohibits this
		targetVelocity *= alpha * 0.5;
	}

	return targetVelocity;
}

void MoveItPlayer::setupReflexxes()
{
	if (! currentKeyFrame())
		return;
	if(m_keyframeIndex >= m_motion->size())
	{
		return;
	}

	m_state->updateLinkTransforms();

	m_jointSpaceVelocityLimit = 0;

	const KeyFrame* frame = currentKeyFrame();

	unsigned int dofs = 0;
	KeyFrame::GroupMap::const_iterator group_it =frame->jointGroups().begin();
	for(; group_it != frame->jointGroups().end(); ++group_it)
	{
		std::string groupName = group_it->first;
		Group* group = &m_groups.at(groupName);

		switch(group_it->second.interpolationSpace())
		{
			case KeyFrame::IS_CARTESIAN:
			{
				dofs += 4;

				// Get the state of the end effector of this group
				auto srdf = m_model->getSRDF();
				std::string tipFrame;

				for(auto eef : srdf->getEndEffectors())
				{
					if(eef.parent_group_ == groupName)
					{
						tipFrame = eef.parent_link_;
						break;
					}
				}

				if(tipFrame.empty())
				{
					ROS_ERROR_STREAM("No end effector link is specified for the group: "<<groupName);
					throw std::runtime_error("No endeffector link");
				}

				Eigen::Affine3d currentState = m_state->getGlobalLinkTransform(tipFrame);

				group->endEffectorPosition << currentState.translation(), 0;

				if(group->interpolationSpace != KeyFrame::IS_CARTESIAN)
				{
// 					ROS_ERROR("%s: ================== RESETTING VELOCITY =========================, last interpolation space: %d", groupName.c_str(), group->interpolationSpace);
					group->endEffectorVelocity.setZero();
					group->endEffectorAcceleration.setZero();
				}

				// either calculate the target velocity or set it so zero
				if(frame->continuous())
				{
					group->endEffectorTargetVelocity = calculateTargetVelocity(
						groupName,
						currentState.translation(),
						frame->state(groupName).translation(),
						group->endEffectorVelocity,
						frame->linearVelocity()
					);
				}
				else
				{
					group->endEffectorTargetVelocity.setZero();
				}

				group->rotationStart = currentState.rotation();
				group->rotationEnd = group_it->second.state().rotation();

				switch(group->rotationSpace)
				{
					case Group::ROTSPACE_FULL:
						group->rotationDist = group->rotationEnd.angularDistance(group->rotationStart);
						break;
					case Group::ROTSPACE_WITHOUT_ROLL:
					{
						Eigen::Vector3d start = group->rotationStart * Eigen::Vector3d::UnitX();
						Eigen::Vector3d end = group->rotationEnd * Eigen::Vector3d::UnitX();
						group->rotationDist = std::abs(robustAcos(start.dot(end)));
						break;
					}
					case Group::ROTSPACE_WITHOUT_PITCH:
					{
						Eigen::Vector3d start = group->rotationStart * Eigen::Vector3d::UnitY();
						Eigen::Vector3d end = group->rotationEnd * Eigen::Vector3d::UnitY();
						group->rotationDist = std::abs(robustAcos(start.dot(end)));
						break;
					}
					case Group::ROTSPACE_WITHOUT_YAW:
					{
						Eigen::Vector3d start = group->rotationStart * Eigen::Vector3d::UnitZ();
						Eigen::Vector3d end = group->rotationEnd * Eigen::Vector3d::UnitZ();
						group->rotationDist = std::abs(robustAcos(start.dot(end)));
						break;
					}
				}

				// Copy current state into the IK state
				*m_ikState = *m_state;
// 				ROS_INFO_STREAM ("Current State : \n"  << group_it->second.state().matrix());

// 				ROS_INFO_STREAM("Cartesian interpolation starting at " << group->endEffectorPosition.transpose());
				break;
			}
			case KeyFrame::IS_JOINT_SPACE:
			{
				unsigned int groupDofs = group->jointGroup->getVariableCount();
				dofs += groupDofs;

				group->jointSpacePositions.resize(groupDofs);
				group->jointSpaceVelocities.resize(groupDofs);
				group->jointSpaceAccelerations.resize(groupDofs);

				m_state->copyJointGroupPositions(group->jointGroup->getName(), group->jointSpacePositions.data());

				if(group->interpolationSpace != KeyFrame::IS_JOINT_SPACE)
				{
					group->jointSpaceVelocities.setZero();
					group->jointSpaceAccelerations.setZero();
				}

				break;
			}
			case KeyFrame::IS_NONE:
				continue;
		}
	}

	for(std::map<std::string, Group>::iterator it = m_groups.begin(); it != m_groups.end(); ++it)
		it->second.interpolationSpace = KeyFrame::IS_NONE;

	for(group_it = frame->jointGroups().begin(); group_it != frame->jointGroups().end(); ++group_it)
	{
		std::string groupName = group_it->first;
		Group* group = &m_groups.at(groupName);

		ROS_INFO("%s: Setting interpolation space: %d", groupName.c_str(), group_it->second.interpolationSpace());
		group->interpolationSpace = group_it->second.interpolationSpace();
	}

	m_reflexxes.reset(new ReflexxesAPI(dofs, m_dt.toSec()));
	m_reflexxesInput.reset(new RMLPositionInputParameters(dofs));
	m_reflexxesOutput.reset(new RMLPositionOutputParameters(dofs));

	if(m_startCallback)
		m_startCallback(*currentKeyFrame());
}

void MoveItPlayer::trajectoryHandleGoal(const TrajectoryGoalHandle& goal)
{
	if(m_trajectoryGoal.isValid())
	{
		ROS_INFO("Preempting active trajectory goal");
		m_trajectoryGoal.setCanceled();
	}

	m_firstExec = true;
	m_trajectoryPoint = 0;
	m_trajectoryGoal = goal;
	m_trajectoryGoal.setAccepted();
}

void MoveItPlayer::trajectoryCancelGoal(TrajectoryGoalHandle goal)
{
	if(goal == m_trajectoryGoal)
		m_trajectoryGoal = TrajectoryGoalHandle();

	goal.setCanceled();
}

void MoveItPlayer::useInternalCollisionChecking(bool use)
{
	m_useCollisionChecking = use;
}

bool MoveItPlayer::step(nimbro_keyframe_server::ExecutionFeedback* feedback)
{
	// backup the current state
	*m_unmodifiedRobotState = *m_state;

	bool result = executeReflexxes(feedback);
	bool revert = false;

	if(m_collisionDetection() && m_useCollisionChecking)
	{
		if(!m_collisionDetectionSrv.exists())
		{
			ROS_ERROR("Could not check for collisions.");
			revert = true;
		}
		else
		{
			nimbro_collision_detection::CollisionDetection srv;
			moveit::core::robotStateToJointStateMsg(*m_state, srv.request.state);

			if(!m_collisionDetectionSrv.call(srv))
			{
				ROS_ERROR("Could not check for collisions.");
				revert = true;
			}

			bool inCollision = (srv.response.distance <= 0.0);
			if(inCollision)
			{
				for(auto& pair: srv.response.pairs)
				{
					ROS_WARN("collision between: '%s' and '%s'.",pair.first.c_str(), pair.second.c_str());
				}
				revert = true;
				m_inCollision = true;
			}
		}

		if(revert)
		{
			ROS_WARN("Stopping execution. Robot is nearly in collision.");

			// revert the changes
			*m_state = *m_unmodifiedRobotState;

			// stop the current motion
			m_motion.reset();
			result = false;
		}
	}

	return result;
}

bool MoveItPlayer::executeReflexxes(nimbro_keyframe_server::ExecutionFeedback* feedback)
{
	if(m_trajectoryGoal.isValid())
	{
		auto trajectory = m_trajectoryGoal.getGoal()->trajectory;

		if(m_trajectoryPoint < trajectory.points.size())
		{
			auto point = trajectory.points[m_trajectoryPoint];

			if(m_firstExec)
			{
				m_firstExec = false;
				m_execTime = ros::Time::now();
			}

			if((ros::Time::now()-m_execTime).toNSec() >= point.time_from_start.toNSec())
			{
				for(unsigned int j=0; j < trajectory.joint_names.size(); ++j)
				{
					m_state->setVariableAcceleration(trajectory.joint_names[j], 0.0 /*m_trajectory.points[0].accelerations[j]*/);
					m_state->setVariableVelocity(trajectory.joint_names[j], point.velocities[j]);
					m_state->setVariablePosition(trajectory.joint_names[j], point.positions[j]);
				}

				m_trajectoryPoint++;
			}
		}
		else
		{
			control_msgs::FollowJointTrajectoryResult rs;
			rs.error_code = rs.SUCCESSFUL;
			m_trajectoryGoal.setSucceeded(rs);
			m_trajectoryGoal = TrajectoryGoalHandle();
		}

		if(feedback)
			feedback->playing = false;

		return false;
	}

	if(!m_motion)
	{
		if(feedback)
			feedback->playing = false;

		return false;
	}

	if(m_keyframeIndex >= m_motion->size() || m_interrupt)
	{
		m_motion.reset();

		if(feedback)
			feedback->playing = false;

		return false;
	}

	if(feedback)
	{
		feedback->playing = true;
		feedback->keyframe_index = m_keyframeIndex;
	}

	const KeyFrame* frame = currentKeyFrame();

	// Accelerate with m_maxJointSpaceAcceleration until we reach the velocity
	// limit
	double currentMaxJointSpaceVelocity = std::isfinite(frame->jointSpaceVelocity()) ? frame->jointSpaceVelocity() : m_maxJointSpaceVelocity();
	double currentMaxJointSpaceAcceleration = std::isfinite(frame->jointSpaceAcceleration()) ? frame->jointSpaceAcceleration() : m_maxJointSpaceAcceleration();

	if(m_jointSpaceVelocityLimit < currentMaxJointSpaceVelocity)
	{
		m_jointSpaceVelocityLimit = std::min<double>(
			currentMaxJointSpaceVelocity,
			m_jointSpaceVelocityLimit + m_dt.toSec() * currentMaxJointSpaceAcceleration
		);
	}

	KeyFrame::GroupMap::const_iterator group_it = frame->jointGroups().begin();
	unsigned int dof_offset = 0;
	for (; group_it != frame->jointGroups().end(); ++group_it)
	{
		const std::string& groupName = group_it->first;
		Group* group = &m_groups.at(groupName);

		switch(group_it->second.interpolationSpace())
		{
			case KeyFrame::IS_CARTESIAN:
			{
// 				ROS_INFO("dof_offset: %u", dof_offset);
				for(unsigned int j = 0; j < 4; ++j)
				{
// 					ROS_INFO("pos input %d: %f", j, group->endEffectorPosition[j]);
					m_reflexxesInput->CurrentPositionVector->VecData[dof_offset + j] = group->endEffectorPosition[j];
					m_reflexxesInput->CurrentVelocityVector->VecData[dof_offset + j] = group->endEffectorVelocity[j];
					m_reflexxesInput->CurrentAccelerationVector->VecData[dof_offset + j] = group->endEffectorAcceleration[j];
				}

				for (unsigned int j = 0; j < 3; ++j)
				{
					m_reflexxesInput->TargetPositionVector->VecData[dof_offset + j] = frame->state(groupName).translation()[j];
					m_reflexxesInput->TargetVelocityVector->VecData[dof_offset + j] = group->endEffectorTargetVelocity(j);

					m_reflexxesInput->MaxVelocityVector->VecData[dof_offset + j] = std::isfinite(frame->linearVelocity()) ?
						frame->linearVelocity() : m_maxLinearVelocity();
					m_reflexxesInput->MaxAccelerationVector->VecData[dof_offset + j] = std::isfinite(frame->linearAcceleration()) ?
						frame->linearAcceleration() : m_maxLinearAcceleration();
					m_reflexxesInput->MaxJerkVector->VecData[dof_offset + j] = m_maxJerk();
					m_reflexxesInput->SelectionVector->VecData[dof_offset + j] = true;
				}

				m_reflexxesInput->TargetPositionVector->VecData[dof_offset + 3] = group->rotationDist;
				m_reflexxesInput->MaxVelocityVector->VecData[dof_offset + 3] = std::isfinite(frame->angularVelocity()) ?
					frame->angularVelocity() : m_maxAngularVelocity();
				m_reflexxesInput->MaxAccelerationVector->VecData[dof_offset + 3] = std::isfinite(frame->angularAcceleration()) ?
					frame->angularAcceleration() : m_maxAngularAcceleration();
				m_reflexxesInput->MaxJerkVector->VecData[dof_offset + 3] = m_maxJerk();
				m_reflexxesInput->SelectionVector->VecData[dof_offset + 3] = true;
				dof_offset += 4;
				break;
			}
			case KeyFrame::IS_JOINT_SPACE:
			{
				std::vector<std::string> gJoints = group->jointGroup->getVariableNames();
				unsigned int varCount = group->jointGroup->getVariableCount();
				for(unsigned int j = 0; j < varCount; ++j)
				{
					m_reflexxesInput->CurrentPositionVector->VecData[dof_offset + j] = group->jointSpacePositions[j];
					m_reflexxesInput->CurrentVelocityVector->VecData[dof_offset + j] = group->jointSpaceVelocities[j];
					m_reflexxesInput->CurrentAccelerationVector->VecData[dof_offset + j] = group->jointSpaceAccelerations[j];

					// Target position

					std::map<std::string, double>::const_iterator it = frame->jointPositions().find(gJoints[j]);
					if(it == frame->jointPositions().end())
						m_reflexxesInput->TargetPositionVector->VecData[dof_offset + j] = group->jointSpacePositions[j];
					else
						m_reflexxesInput->TargetPositionVector->VecData[dof_offset + j] = it->second;

					m_reflexxesInput->MaxVelocityVector->VecData[dof_offset + j] = std::isfinite(frame->jointSpaceVelocity()) ?
						frame->jointSpaceVelocity() : m_maxJointSpaceVelocity();

					m_reflexxesInput->MaxAccelerationVector->VecData[dof_offset + j] = std::isfinite(frame->jointSpaceAcceleration()) ?
							frame->jointSpaceAcceleration() : m_maxJointSpaceAcceleration();

					m_reflexxesInput->MaxJerkVector->VecData[dof_offset + j] = m_maxJerk();
					m_reflexxesInput->SelectionVector->VecData[dof_offset + j] = true;
				}
				dof_offset += varCount;
				break;
			}
			case KeyFrame::IS_NONE:
				continue;
		}
	}

	// Reflexxes call
	int ret = m_reflexxes->RMLPosition(*m_reflexxesInput, m_reflexxesOutput.get(), m_reflexxesFlags);

	m_state->updateLinkTransforms();

// 	ROS_INFO("Reflexxes: %d", ret);

	dof_offset = 0;

	double maxJointSpaceVelocity = 0.0f;

	if(feedback)
		feedback->active_limits = 0;

	// Reflexxes output
	group_it = frame->jointGroups().begin();
	dof_offset = 0;
	for (; group_it != frame->jointGroups().end(); ++group_it)
	{
		const std::string& groupName = group_it->first;
		Group* group = &m_groups[groupName];

		switch(group_it->second.interpolationSpace())
		{
			case KeyFrame::IS_CARTESIAN:
			{
				// We need to use the result of the Reflexxes call in this iteration
				Eigen::Vector4d nextEndEffectorPosition = Eigen::Map<Eigen::Vector4d>(m_reflexxesOutput->NewPositionVector->VecData + dof_offset);
				Eigen::Vector4d nextEndEffectorVelocity = Eigen::Map<Eigen::Vector4d>(m_reflexxesOutput->NewVelocityVector->VecData + dof_offset);
				Eigen::Vector4d nextEndEffectorAcceleration = Eigen::Map<Eigen::Vector4d>(m_reflexxesOutput->NewAccelerationVector->VecData + dof_offset);
				group->endEffectorPosition = nextEndEffectorPosition;
				group->endEffectorVelocity = nextEndEffectorVelocity;
				group->endEffectorAcceleration = nextEndEffectorAcceleration;

				// Check if we are close to vel/acc limits for operator feedback
				if(feedback)
				{
					double linVel = group->endEffectorVelocity.head<3>().array().abs().maxCoeff();
					double linAcc = group->endEffectorAcceleration.head<3>().array().abs().maxCoeff();
					double angVel = std::abs(group->endEffectorVelocity[3]);
					double angAcc = std::abs(group->endEffectorAcceleration[3]);

					if(std::abs(linVel - m_reflexxesInput->MaxVelocityVector->VecData[dof_offset]) < 1e-2)
						feedback->active_limits |= ExecutionFeedback::LIN_VEL_LIMIT;
					if(std::abs(linAcc - m_reflexxesInput->MaxAccelerationVector->VecData[dof_offset]) < 1e-2)
						feedback->active_limits |= ExecutionFeedback::LIN_ACC_LIMIT;
					if(std::abs(angVel - m_reflexxesInput->MaxVelocityVector->VecData[dof_offset + 3]) < 1e-2)
						feedback->active_limits |= ExecutionFeedback::ANG_VEL_LIMIT;
					if(std::abs(angAcc - m_reflexxesInput->MaxAccelerationVector->VecData[dof_offset + 3]) < 1e-2)
						feedback->active_limits |= ExecutionFeedback::ANG_ACC_LIMIT;
				}

				// IK
				Eigen::Affine3d state;
				state = Eigen::Translation3d(group->endEffectorPosition.head<3>());

				if(fabs(group->rotationDist) > 1e-4)
				{
					state = state * group->rotationStart.slerp(group->endEffectorPosition[3] / group->rotationDist, group->rotationEnd);
				}
				else
				{
					state = state * group->rotationEnd;
				}

				*m_ikState = *m_state;

// 				ROS_INFO_STREAM("interpolated state: " << state.translation().transpose() << ", euler: " << state.rotation().eulerAngles(0,1,2).transpose());

				if(!m_ikState->setFromIK(group->jointGroup, state) )
				{
					if(!m_skipStep)
					{
						ROS_ERROR("IK failed at keyframe %d, stopping motion...", m_keyframeIndex);
						ROS_ERROR_STREAM("state was: " << state.translation().transpose() << ", euler: " << state.rotation().eulerAngles(0,1,2).transpose());
						m_motion.reset();
						m_inCollision = true;
						return true;
					}
					// if skip is possible try with the next keyframe
					ROS_WARN("********************    Skipping Step     ******************** ");
					m_keyframeIndex++;
					return false;
				}

				Eigen::VectorXd oldPos(group->jointGroup->getVariableCount());
				m_state->copyJointGroupPositions(group->jointGroup, oldPos.data());

				Eigen::VectorXd newPos(group->jointGroup->getVariableCount());
				m_ikState->copyJointGroupPositions(group->jointGroup, newPos.data());

				Eigen::VectorXd jointDelta = newPos - oldPos;

				double maxDelta = jointDelta.array().abs().maxCoeff();
				double maxVel = maxDelta / m_dt.toSec();

				if(maxVel > m_jointSpaceVelocityLimit)
				{
					// Scale jointDelta back so that we are within our limits
					double alpha = m_jointSpaceVelocityLimit / maxVel;

					jointDelta = alpha * jointDelta;
					maxVel = alpha * maxVel;

					// Taylor approximation
					nextEndEffectorPosition = group->endEffectorPosition + alpha * (nextEndEffectorPosition - group->endEffectorPosition);
					nextEndEffectorVelocity = alpha * nextEndEffectorVelocity;
					nextEndEffectorAcceleration = alpha * nextEndEffectorAcceleration;

					// Write joint values
					Eigen::VectorXd nextJointAngles = oldPos + jointDelta;
					m_state->setJointGroupPositions(group->jointGroup, nextJointAngles.data());

					m_limitingJointSpaceVelocity = true;

					if(feedback)
						feedback->active_limits |= ExecutionFeedback::CART_JS_VEL_LIMIT;
				}
				else
				{
					m_state->setJointGroupPositions(group->jointGroup, newPos.data());
					m_limitingJointSpaceVelocity = false;
				}

				maxJointSpaceVelocity = std::max(maxVel, maxJointSpaceVelocity);

				Eigen::VectorXd jointVelocity = jointDelta / m_dt.toSec();
				{
					const std::vector<int> &il = group->jointGroup->getVariableIndexList();
					for (std::size_t i = 0 ; i < il.size() ; ++i)
					{
						m_state->setVariableVelocity(il[i], jointVelocity[i]);
					}
				}

				// Update reflexxes state so the motion continues.
				group->endEffectorPosition = nextEndEffectorPosition;
				group->endEffectorVelocity = nextEndEffectorVelocity;
				group->endEffectorAcceleration = nextEndEffectorAcceleration;

				dof_offset += 4;
				break;
			}
			case KeyFrame::IS_JOINT_SPACE:
			{
				std::vector<std::string> gJoints = group->jointGroup->getVariableNames();
				int varCount  = group->jointGroup->getVariableCount();
				for (int j = 0; j < varCount; ++j)
				{
					group->jointSpacePositions[j] = m_reflexxesOutput->NewPositionVector->VecData[j + dof_offset];
					group->jointSpaceVelocities[j] = m_reflexxesOutput->NewVelocityVector->VecData[j + dof_offset];
					group->jointSpaceAccelerations[j] = m_reflexxesOutput->NewAccelerationVector->VecData[j + dof_offset];

					maxJointSpaceVelocity = std::max(std::abs(group->jointSpaceVelocities[j]), maxJointSpaceVelocity);

					// Check if we are close to vel/acc limits for operator feedback
					if(feedback)
					{
						if(std::abs(group->jointSpaceVelocities[j] - m_reflexxesInput->MaxVelocityVector->VecData[j + dof_offset]) < 1e-2)
							feedback->active_limits |= ExecutionFeedback::JS_VEL_LIMIT;
						if(std::abs(group->jointSpaceAccelerations[j] - m_reflexxesInput->MaxAccelerationVector->VecData[j + dof_offset]) < 1e-2)
							feedback->active_limits |= ExecutionFeedback::JS_ACC_LIMIT;
					}
				}
				m_state->setJointGroupPositions(group->jointGroup, group->jointSpacePositions.data());

				{
					const std::vector<int> &il = group->jointGroup->getVariableIndexList();
					for (std::size_t i = 0 ; i < il.size() ; ++i)
						m_state->setVariableVelocity(il[i],group->jointSpaceVelocities[i]);
				}

				dof_offset += varCount;
				break;
			}
			case KeyFrame::IS_NONE:
				continue;
		}
	}

	if(m_keyframeIndex + 1 >= m_motion->size())
	{
		// We are in the last keyframe of the motion. Make sure that we are at
		// complete rest before ending the trajectory. In particular, the
		// IK might still be performing nullspace optimization.

		if(maxJointSpaceVelocity > m_maxFinalJointSpaceVelocity() * M_PI / 180.0)
			return false; // not finished
	}

	if(ret == ReflexxesAPI::RML_FINAL_STATE_REACHED)
	{
// 		ROS_INFO_STREAM("ReflexxesAPI::RML_FINAL_STATE_REACHED");

		if(m_endCallback)
			m_endCallback(*currentKeyFrame());

		m_keyframeIndex++;
		setupReflexxes();


		if(m_keyframeIndex >= m_motion->size())
		{
			// the final state of the last keyframe was reached
			return true;
		}
	}

	return false;
}

void MoveItPlayer::setStartCallback(const Callback& cb)
{
	m_startCallback = cb;
}

void MoveItPlayer::setEndCallback(const Callback& cb)
{
	m_endCallback = cb;
}

Eigen::Vector4d MoveItPlayer::getEndEffectorPosition(std::string groupName)
{
	auto it=m_groups.find(groupName);
	if(it==m_groups.end())
	{
		ROS_ERROR("Can't find group '%s'.",groupName.c_str());
		return Eigen::Vector4d().setZero();
	}
		return m_groups[groupName].endEffectorPosition;
}

Eigen::Vector4d MoveItPlayer::getEndEffectorVelocity(std::string groupName)
{
	auto it=m_groups.find(groupName);
	if(it==m_groups.end())
	{
		ROS_ERROR("Can't find group '%s'.",groupName.c_str());
		return Eigen::Vector4d().setZero();
	}
	return m_groups[groupName].endEffectorVelocity;
}


Eigen::Vector4d MoveItPlayer::getEndEffectorAcceleration(std::string groupName)
{
	auto it=m_groups.find(groupName);
	if(it==m_groups.end())
	{
		ROS_ERROR("Can't find group '%s'.",groupName.c_str());
		return Eigen::Vector4d().setZero();
	}
	return m_groups[groupName].endEffectorAcceleration;
}



void MoveItPlayer::debugDump()
{
	ROS_INFO("Current MoveItPlayer state:");
	for(auto& group : m_groups)
	{
		ROS_INFO(" - Group %s:", group.first.c_str());
		ROS_INFO_STREAM(" endEffectorPosition:\n" << group.second.endEffectorPosition.matrix());
		ROS_INFO_STREAM(" jointSpacePositions: " << group.second.jointSpacePositions.transpose());
		ROS_INFO_STREAM(" rotationStart: " << group.second.rotationStart.coeffs().transpose());
		ROS_INFO_STREAM(" rotationEnd: " << group.second.rotationEnd.coeffs().transpose());
		ROS_INFO_STREAM(" rotationDist: " << group.second.rotationDist);
	}

	if(m_keyframeIndex < m_motion->size())
	{
		ROS_INFO("Current MoveItPlayer keyframe:");
		ROS_INFO("%s", m_motion->serializeToYAML().c_str());
	}
}

}
