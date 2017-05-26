// Interface to ROS control (FollowJointTrajectory action)
// Author: Max Schwarz <max.schwarz@uni-bonn.de>
// Author: Sebastian Schüller <schuell1@cs.uni-bonn.de>

#ifndef ROS_CONTROL_PLAYER_H
#define ROS_CONTROL_PLAYER_H

#include <ros/init.h>
#include <nimbro_keyframe_server/motion.h>

#include <nimbro_keyframe_server/moveit_player.h>
#include <nimbro_keyframe_server/keyframe_server.h>
#include <nimbro_keyframe_server/utils.h>

#include <trajectory_score/trajectory_score.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

#include <nimbro_keyframe_server/PlayMotionAction.h>

#include <control_msgs/FollowJointTrajectoryAction.h>

#include <config_server/parameter.h>

#include <thread>
#include <mutex>
#include <condition_variable>

namespace nimbro_keyframe_server
{

class ROSControlPlayer
{
public:
	ROSControlPlayer();
	virtual ~ROSControlPlayer();

	bool init();

	bool play(const nimbro_keyframe_server::Motion::Ptr& motion);

protected:
	void abortTrajectoryGoal(uint8_t code, const std::string& error_string);

private:
	bool computeTrajectory(const nimbro_keyframe_server::Motion::Ptr& motion);
	void splitTrajectory(const trajectory_msgs::JointTrajectory& trajectory);

	std::string errorToString(int error_code);

	ros::NodeHandle m_nh;
	nimbro_keyframe_server::KeyFrameServer m_keyframeServer;

	boost::shared_ptr<robot_model_loader::RobotModelLoader> m_moveitLoader;
	robot_model::RobotModelPtr m_robotModel;
	robot_state::RobotStatePtr m_playerState;

	std::mutex m_robotStateMutex;
	std::condition_variable m_robotStateUpdate;
	boost::shared_ptr<robot_state::RobotState> m_robotState;
	ros::Time m_robotStateStamp;

	std::unique_ptr<nimbro_keyframe_server::MoveItPlayer> m_player;

	struct JointInfo
	{
		std::string actionPath;
		double pathTolerance;
		double goalTolerance;
	};
	std::vector<std::string> m_whitelist;
	std::map<std::string, JointInfo> m_jointsActionMap;

	ros::Duration m_period;
	ros::Duration m_timeTolerance;
	double m_pathTolerance;
	double m_goalTolerance;
	config_server::Parameter<float> m_collisionPadding;
	config_server::Parameter<bool> m_collisionChecking;
	config_server::Parameter<int> m_collisionSubsampling;
	config_server::Parameter<float> m_interpolationPeriod;

	ros::Duration m_est_motion_duration;
	ros::Time m_motion_started;

	ros::CallbackQueue m_jointStateCallbackQueue;
	ros::AsyncSpinner m_jointStateSpinner;
	ros::NodeHandle m_jointStateNH;

	ros::Timer m_feedbackTimer;

	//! @name PlayMotion Action *server*
	//! @brief SimpleActionServer interface to interpolate and play
	//    Motions. Interpolation will only check for self collisions. For
	//    more sophisticated behaviour, do the interpolation yourself and
	//    use the FollowJointTrajectory SimpleActionServer instead.
	//@{
	using PlayAction = nimbro_keyframe_server::PlayMotionAction;
	using PlayResult = nimbro_keyframe_server::PlayMotionResult;
	using PlayMotionServer = actionlib::ActionServer<PlayAction>;
	using PlayGoalHandle = PlayMotionServer::GoalHandle;

	boost::shared_ptr<PlayMotionServer> m_act_playMotionServer;

	void playMotionServerHandleGoal(const PlayGoalHandle& goal);
	void playMotionServerCancelGoal(PlayGoalHandle goal);
	void publishPlayMotionServerFeedback();

	PlayResult m_playMotionResult;

	enum class PlayMotionStatus
	{
		PLAYING,
		PREEMPTING,

		SUCCEEDED,
		ABORTED,
		PREEMPTED
	};

	PlayMotionStatus m_playMotionStatus;

	// must be called with m_playMotionResultMutex held!
	void playMotionServerReportResult(const PlayResult& result, PlayMotionStatus status);
	void playMotionServerSwitchGoal();

	PlayGoalHandle m_playMotionGoal;
	PlayGoalHandle m_playMotionNextGoal;
	//@}

	//! @name modifyMotion
	//! @brief Allow subclasses to modify motions before playing
	virtual void modifyMotion(nimbro_keyframe_server::Motion::Ptr motion);


	//! @name FollowJointTrajectory Action *server*
	//! @brief SimpleActionServer interface to execute already interpolated
	//    Motions.
	//@{
	using TrajectoryAction = control_msgs::FollowJointTrajectoryAction;
	using TrajectoryServer = actionlib::SimpleActionServer<TrajectoryAction>;
	boost::shared_ptr<TrajectoryServer> m_act_trajectoryServer;

	void trajectoryServerHandleGoal();
	void trajectoryServerCancelGoal();
	//@}


	//! @name FollowJointTrajectory Action *client*
	//@{
	using TrajectoryClient = actionlib::SimpleActionClient<TrajectoryAction>;
	std::map<std::string, std::shared_ptr<TrajectoryClient> > m_trajectoryClients;
	std::map<std::string, control_msgs::FollowJointTrajectoryGoal> m_goals;

	void trajectoryClientHandleFinished(
		const actionlib::SimpleClientGoalState& state,
		const control_msgs::FollowJointTrajectoryResultConstPtr& result,
		const std::string& actionName
	);

	void trajectoryClientHandleActive(std::string actionName);
	
	void trajectoryClientHandleProblem(
		const std::string& action,
		const actionlib::SimpleClientGoalState& state,
		const control_msgs::FollowJointTrajectoryResultConstPtr& result
	);
	void trajectoryClientsCancelAllGoals();
	//@}

	//! @name Joint State Subscriber
	//@{
	ros::Subscriber m_sub_js;
	void handleJointStates(const sensor_msgs::JointStateConstPtr& msg);
	//@}

	// DEPRECATED (sebastian): This only gets called, iff the a motion is
	// triggered via a service call to the Service-Server inside the
	// moveit player.
	//@{
	void playServiceHandleActionFinished(const std::string& name, const actionlib::SimpleClientGoalState& state,
		const control_msgs::FollowJointTrajectoryResultConstPtr& result);
	//@}
};

}

#endif
