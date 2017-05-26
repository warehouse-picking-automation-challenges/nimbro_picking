// Motion player for MoveIt
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef MOVEIT_PLAYER_H
#define MOVEIT_PLAYER_H

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <nimbro_keyframe_server/motion.h>
#include <nimbro_keyframe_server/ExecutionFeedback.h>

#include <config_server/parameter.h>

#include <reflexxes/ReflexxesAPI.h>
#include <reflexxes/RMLPositionFlags.h>
#include <reflexxes/RMLPositionInputParameters.h>
#include <reflexxes/RMLPositionOutputParameters.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <boost/function.hpp>

#include <std_msgs/Float32.h>



namespace nimbro_keyframe_server
{

class MoveItPlayer
{
public:
	typedef boost::function<void(const nimbro_keyframe_server::KeyFrame&)> Callback;

	MoveItPlayer(
		const robot_model::RobotModelPtr& model,
		const robot_state::RobotStatePtr& state,
		ros::Duration dt
	);

	~MoveItPlayer();

	// returns true if the final state of the motion is reached
	bool step(nimbro_keyframe_server::ExecutionFeedback* feedback = 0);

	bool playMotion(const Motion::Ptr& motion);
	void stopMotion();

	inline bool limitingJointSpaceVelocity() const
	{ return m_limitingJointSpaceVelocity; }

	void setStartCallback(const Callback& cb);
	void setEndCallback(const Callback& cb);

	void useInternalCollisionChecking(bool use);
	bool inCollision()
	{ return m_inCollision; }

	Eigen::Vector4d getEndEffectorPosition(std::string groupName);
	Eigen::Vector4d getEndEffectorVelocity(std::string groupName);
	Eigen::Vector4d getEndEffectorAcceleration(std::string groupName);

	void debugDump();

private:
	struct Group
	{
		Eigen::Vector4d endEffectorPosition;
		Eigen::Vector4d endEffectorVelocity;
		Eigen::Vector4d endEffectorAcceleration;

		Eigen::Vector3d endEffectorTargetVelocity;

		Eigen::VectorXd jointSpacePositions;
		Eigen::VectorXd jointSpaceVelocities;
		Eigen::VectorXd jointSpaceAccelerations;

		Eigen::Quaterniond rotationStart;
		Eigen::Quaterniond rotationEnd;
		double rotationDist;

		const robot_model::JointModelGroup* jointGroup;

		enum RotationSpace
		{
			ROTSPACE_FULL,
			ROTSPACE_WITHOUT_ROLL,
			ROTSPACE_WITHOUT_PITCH,
			ROTSPACE_WITHOUT_YAW,
		};

		RotationSpace rotationSpace;

		nimbro_keyframe_server::KeyFrame::InterpolationSpace interpolationSpace;
	};

	inline const KeyFrame* currentKeyFrame()
	{ return &(*m_motion)[m_keyframeIndex]; }
	void setupReflexxes();
	bool executeReflexxes(nimbro_keyframe_server::ExecutionFeedback* feedback = 0);

	robot_model::RobotModelPtr m_model;
	robot_state::RobotStatePtr m_state;
	robot_state::RobotStatePtr m_unmodifiedRobotState;
	robot_state::RobotStatePtr m_ikState;
	ros::Duration m_dt;

	std::map<std::string, Group> m_groups;

	Motion::Ptr m_motion;
	unsigned int m_keyframeIndex;

	config_server::Parameter< float > m_maxLinearAcceleration;
	config_server::Parameter< float > m_maxAngularAcceleration;
	config_server::Parameter< float > m_maxJointSpaceAcceleration;

	config_server::Parameter< float > m_maxLinearVelocity;
	config_server::Parameter< float > m_maxAngularVelocity;
	config_server::Parameter< float > m_maxJointSpaceVelocity;
	config_server::Parameter< float > m_maxFinalJointSpaceVelocity;

	config_server::Parameter< float > m_maxJerk;
	config_server::Parameter< bool > m_collisionDetection;

	double m_jointSpaceVelocityLimit;

	boost::shared_ptr<ReflexxesAPI> m_reflexxes;
	boost::shared_ptr<RMLPositionInputParameters> m_reflexxesInput;
	boost::shared_ptr<RMLPositionOutputParameters> m_reflexxesOutput;
	RMLPositionFlags m_reflexxesFlags;
	bool m_skipStep;
	bool m_interrupt;
	std::vector<double> m_oldState,m_values;
	std::vector<std::string> m_names;

	using TrajectoryAction = control_msgs::FollowJointTrajectoryAction;
	using TrajectoryResult = control_msgs::FollowJointTrajectoryResult;
	using TrajectoryServer = actionlib::ActionServer<TrajectoryAction>;
	using TrajectoryGoalHandle = TrajectoryServer::GoalHandle;
	boost::shared_ptr<TrajectoryServer> m_trajectoryServer;

	TrajectoryGoalHandle m_trajectoryGoal;
	unsigned int m_trajectoryPoint;

	void trajectoryHandleGoal(const TrajectoryGoalHandle& goal);
	void trajectoryCancelGoal(TrajectoryGoalHandle goal);

	bool m_firstExec;
	ros::Time m_execTime;

	bool m_limitingJointSpaceVelocity;

	Callback m_startCallback;
	Callback m_endCallback;

	Eigen::Vector3d calculateTargetVelocity(
		std::string groupName,
		Eigen::Vector3d currentPosition,
		Eigen::Vector3d desiredPosition,
		Eigen::Vector4d currentVelocity,
		double linearVelocity
	);

	ros::ServiceClient m_collisionDetectionSrv;

	bool m_inCollision;
	// NOTE(sebastian): Second flag toggling internal collision
	// checking. Seems a bit hacky, but allows to toggle checking
	// per moveit player instance.
	bool m_useCollisionChecking;
};

}

#endif
