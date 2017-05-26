// Interface to ROS control (FollowJointTrajectory action)
// Author: Max Schwarz <max.schwarz@uni-bonn.de>
// Author: Sebastian Schüller <schuell1@cs.uni-bonn.de>

#include <ros_control_player/ros_control_player.h>

namespace nimbro_keyframe_server
{

ROSControlPlayer::ROSControlPlayer()
 : m_nh("~")
 , m_collisionPadding("/ros_player/collision_padding", 0.0f, 0.001f, 0.05f, 0.02f)
 , m_collisionChecking("/ros_player/collision_checking", true)
 , m_collisionSubsampling("/ros_player/collsion_subsampling", 0, 1, 10, 1)
 , m_interpolationPeriod("/ros_player/interpolation_period", 0.02f, 0.005f, 0.1f, 0.05f)
 , m_jointStateSpinner(0, &m_jointStateCallbackQueue)
{
	double rate;

	m_nh.param("path_tolerance", m_pathTolerance, 10.0 * M_PI / 180.0);
	m_nh.param("goal_tolerance", m_goalTolerance, 5.0 * M_PI / 180.0);
	m_nh.param("rate", rate, 20.0);
	m_period = ros::Duration(1.0 / rate);

	double timeToleranceS;
	m_nh.param("time_tolerance", timeToleranceS, 1.0);
	m_timeTolerance = ros::Duration(timeToleranceS);
}

ROSControlPlayer::~ROSControlPlayer()
{
	m_jointStateSpinner.stop();
	m_robotStateUpdate.notify_all();
}

void ROSControlPlayer::handleJointStates(const sensor_msgs::JointStateConstPtr& msg)
{
	if(msg->name.size() != msg->position.size())
	{
		ROS_ERROR("Invalid joint_states msg");
		return;
	}

	{
		std::unique_lock<std::mutex> lock(m_robotStateMutex);
		m_robotState->setVariableValues(*msg);
		m_robotStateStamp = msg->header.stamp;
	}
	m_robotStateUpdate.notify_all();
}

std::string ROSControlPlayer::errorToString(int error_code)
{
	std::string msg;
	switch(error_code){
		case 0: msg="SUCCESSFUL"; break;
		case -1: msg="INVALID_GOAL"; break;
		case -2: msg="INVALID_JOINTS"; break;
		case -3: msg="OLD_HEADER_TIMESTAMP"; break;
		case -4: msg="PATH_TOLERANCE_VIOLATED"; break;
		case -5: msg="GOAL_TOLERANCE_VIOLATED"; break;
		default: {
			std::stringstream ss;
			ss << "invalid error message (" << error_code << ")";
			msg = ss.str();
		}
	};
	return msg;

}

bool ROSControlPlayer::init()
{
	// MoveIt init
	m_moveitLoader = boost::make_shared<robot_model_loader::RobotModelLoader>(
		"robot_description"
	);
	m_robotModel = m_moveitLoader->getModel();

	m_robotState.reset(new robot_state::RobotState(m_robotModel));
	m_robotState->setToDefaultValues();

	for(auto& group : m_robotModel->getJointModelGroups())
	{
		m_robotState->setToDefaultValues(group, "init");
	}

	// Make copy for player
	m_playerState.reset(new robot_state::RobotState(*m_robotState));

	m_player.reset(new nimbro_keyframe_server::MoveItPlayer(
		m_robotModel,
		m_playerState,
		m_period
	));

	m_keyframeServer.setPlayCallback(boost::bind(&ROSControlPlayer::play, this, _1));

	XmlRpc::XmlRpcValue list;
	m_nh.getParam("jointinfo", list);
	if(list.getType() != XmlRpc::XmlRpcValue::TypeArray)
	{
		ROS_ERROR("'jointinfo' param is not an Array!");
		return false;
	}

	for(auto i = 0; i < list.size(); ++i)
	{
		auto& entry = list[i];
		if(entry.getType() != XmlRpc::XmlRpcValue::TypeStruct)
		{
			ROS_ERROR("'jointlist' param entry with id %u is not a struct", i);
			return false;
		}

		if(!entry.hasMember("name"))
		{
			ROS_ERROR("'jointlist' param entry with id %u has no member 'name'", i);
			return false;
		}
		if(!entry.hasMember("action"))
		{
			ROS_ERROR("'jointlist' param entry with id %u has no member 'action'", i);
			return false;
		}
		double goalTolerance = entry.hasMember("goal_tolerance") ?
			static_cast<double>(entry["goal_tolerance"]) : m_goalTolerance;
		double pathTolerance = entry.hasMember("path_tolerance") ?
			static_cast<double>(entry["path_tolerance"]) : m_pathTolerance;
		std::string name   = entry["name"];
		std::string action = entry["action"];

		ROS_INFO("Adding joint '%s'", name.c_str());

		m_jointsActionMap[name].actionPath = action;
		m_jointsActionMap[name].goalTolerance = goalTolerance;
		m_jointsActionMap[name].pathTolerance = pathTolerance;
	}

	// Start Action Clients
	for(const auto& info : m_jointsActionMap)
	{
		auto action = info.second.actionPath;

		auto it = m_trajectoryClients.find(action);
		if(it != m_trajectoryClients.end())
			continue;

		std::shared_ptr<TrajectoryClient> client(
			new TrajectoryClient(m_nh, action, false));
		m_trajectoryClients[action] = client;

		ROS_INFO("Action Client is waiting on action '%s'...", action.c_str());
		if(!utils::waitForActionClient(client.get()))
		{
			ROS_ERROR("... And no action with this name appeared!");
			return false;
		}
		ROS_INFO("... And it appeared");
	}

	// Start Action Servers
	m_act_playMotionServer.reset(new PlayMotionServer(
		m_nh,
		"play_motion",
		boost::bind(&ROSControlPlayer::playMotionServerHandleGoal, this, _1),
		boost::bind(&ROSControlPlayer::playMotionServerCancelGoal, this, _1),
		false
	));
	m_act_playMotionServer->start();

	m_act_trajectoryServer.reset(new TrajectoryServer(
		m_nh, "play_trajectory", false));
	m_act_trajectoryServer->registerGoalCallback(
		boost::bind(&ROSControlPlayer::trajectoryServerHandleGoal, this)
	);
	m_act_trajectoryServer->registerPreemptCallback(
		boost::bind(&ROSControlPlayer::trajectoryServerCancelGoal, this)
	);
	m_act_trajectoryServer->start();

	m_jointStateNH.setCallbackQueue(&m_jointStateCallbackQueue);
	m_sub_js = m_jointStateNH.subscribe("/joint_states", 1, &ROSControlPlayer::handleJointStates, this);

	m_jointStateSpinner.start();

	m_feedbackTimer = m_nh.createTimer(ros::Duration(0.05), boost::bind(&ROSControlPlayer::publishPlayMotionServerFeedback, this));

	ROS_INFO("Ready.");
	return true;
}

void ROSControlPlayer::splitTrajectory(
	const trajectory_msgs::JointTrajectory& trajectory)
{
	m_goals.clear();
	std::vector<std::string> actionClientOrder;
	for(const auto& name : trajectory.joint_names)
	{
		const auto& action = m_jointsActionMap[name].actionPath;
		actionClientOrder.push_back(action);

		using namespace control_msgs;
		JointTolerance goalTolerance, pathTolerance;
		goalTolerance.name = name;
		goalTolerance.position = m_jointsActionMap[name].goalTolerance;
		pathTolerance.name = name;
		pathTolerance.position = m_jointsActionMap[name].pathTolerance;

		m_goals[action].trajectory.joint_names.push_back(name);
		m_goals[action].path_tolerance.push_back(pathTolerance);
		m_goals[action].goal_tolerance.push_back(goalTolerance);
	}
	for(auto& goal : m_goals)
	{
		auto s = trajectory.points.size();
		goal.second.trajectory.points.resize(s);
		goal.second.goal_time_tolerance = m_timeTolerance;
	}
	for(size_t pointIndex = 0;
	    pointIndex < trajectory.points.size();
	    ++pointIndex)
	{
		const auto& point = trajectory.points[pointIndex];
		ROS_ASSERT(point.positions.size() == actionClientOrder.size());
		ROS_ASSERT(point.velocities.size() == actionClientOrder.size());

		for(size_t j = 0; j < actionClientOrder.size(); j++)
		{
			auto& targetPoint =
				m_goals[actionClientOrder[j]].trajectory.points[pointIndex];
			targetPoint.positions.push_back(point.positions[j]);
			targetPoint.velocities.push_back(point.velocities[j]);
			targetPoint.time_from_start = point.time_from_start;
		}
	}

}



/// Play Motion Action Server callbacks
//@{
void ROSControlPlayer::publishPlayMotionServerFeedback()
{
	if(!m_playMotionGoal.isValid())
		return;

	ros::Duration elapsed_motion_time (ros::Time::now() - m_motion_started);
	float time_left = m_est_motion_duration.toSec() - elapsed_motion_time.toSec();

// 	ROS_INFO("est. motion time: %f sec.", time_left);
	nimbro_keyframe_server::PlayMotionFeedback msg;

	msg.estimated_duration = time_left;

	m_playMotionGoal.publishFeedback(msg);
}

void ROSControlPlayer::playMotionServerHandleGoal(const PlayGoalHandle& goal)
{
	using namespace nimbro_keyframe_server;

	if(m_playMotionGoal.isValid())
	{
		ROS_INFO("Current goal is still active");
		ROS_INFO("Triggering child preemption");
		m_playMotionStatus = PlayMotionStatus::PREEMPTING;
		trajectoryClientsCancelAllGoals();

		if(m_playMotionNextGoal.isValid())
		{
			ROS_INFO("Another goal was in the queue, canceling it");
			m_playMotionNextGoal.setCanceled(PlayResult(), "Canceled by newer goal in queue");
		}

		m_playMotionNextGoal = goal;
		return;
	}
	else
	{
		ROS_INFO("I'm currently free.");
		m_playMotionNextGoal = goal;
		playMotionServerSwitchGoal();
	}
}

void ROSControlPlayer::playMotionServerSwitchGoal()
{
	if(m_playMotionGoal.isValid())
		return;

	if(!m_playMotionNextGoal.isValid())
		return;

	// Sanity check: are all clients finished?
	for(auto& goal : m_goals)
	{
		auto& client = m_trajectoryClients[goal.first];

		if(!client->getState().isDone())
		{
			ROS_ERROR("Action client '%s' is still active!", goal.first.c_str());
			client->stopTrackingGoal();
		}
	}

	m_playMotionGoal = m_playMotionNextGoal;
	m_playMotionNextGoal = PlayGoalHandle();

	m_playMotionGoal.setAccepted();
	auto goal = m_playMotionGoal.getGoal();

	// Wait for fresh feedback
	ros::Time startWait = ros::Time::now();

	{
		std::unique_lock<std::mutex> lock(m_robotStateMutex);

		while(ros::ok() && m_robotStateStamp < startWait)
			m_robotStateUpdate.wait(lock);

		*m_playerState = *m_robotState;
	}

	ros::Time endWait = ros::Time::now();
	ROS_INFO("waited %fs for new joint states", (endWait - startWait).toSec());

	ROS_INFO("new action goal: '%s' ",goal->motion_name.c_str());

	boost::shared_ptr<Motion> motion;
	if(goal->use_existing_motion)
	{
		motion = m_keyframeServer.getMotion(goal->motion_name);
		if(!motion)
		{
			ROS_ERROR("invalid motion '%s'", goal->motion_name.empty() ? "" : goal->motion_name.c_str());

			PlayResult new_result;
			new_result.success = false;
			new_result.finish_state = PlayResult::REJECTED;
			new_result.error_code = PlayResult::INVALID_GOAL;

			m_playMotionGoal.setAborted(new_result);
			m_playMotionGoal = PlayGoalHandle();

			return;
		}

		// make copy for adaption
		motion = boost::make_shared<Motion>(*motion);
	}
	else
		motion = boost::make_shared<Motion>(Motion::loadFromMsg(goal->motion_msg));

	modifyMotion(motion);

	auto period = ros::Duration(m_interpolationPeriod());
	trajectory_score::TrajectoryScorer s(
		m_playerState, m_robotModel, period);
	s.setPadding(m_collisionPadding());
	s.setCollisionCheckingEnabled(m_collisionChecking());
	auto score = s.scoreMotion(motion, m_collisionSubsampling());

	if(score.inCollision)
	{
		PlayResult new_result;
		new_result.success = false;
		new_result.finish_state = PlayResult::REJECTED;
		new_result.error_code = PlayResult::COLLISION;
		m_playMotionGoal.setAborted(new_result);
		m_playMotionGoal = PlayGoalHandle();
		ROS_ERROR("Could not compute trajectory!");
		return;
	}

	// Dump trajectory
	{
		char buf[256];
		snprintf(buf, sizeof(buf), "/tmp/trajectory_%s.txt", motion->name().c_str());
		FILE* dump = fopen(buf, "w");

		fprintf(dump, "time");
		for(auto& name : score.trajectory.joint_names)
		{
			fprintf(dump, " %s_pos %s_vel", name.c_str(), name.c_str());
		}
		fprintf(dump, "\n");

		for(auto& step : score.trajectory.points)
		{
			fprintf(dump, "%10.4f ", step.time_from_start.toSec());
			for(size_t i = 0; i < score.trajectory.joint_names.size(); ++i)
			{
				fprintf(dump, " %10.6f %10.6f", step.positions[i], step.velocities[i]);
			}
			fprintf(dump, "\n");
		}

		fclose(dump);
	}

	splitTrajectory(score.trajectory);

	m_playMotionStatus = PlayMotionStatus::PLAYING;

	for(auto& goal : m_goals)
	{
		m_trajectoryClients[goal.first]->sendGoal(
			goal.second,
			boost::bind(&ROSControlPlayer::trajectoryClientHandleFinished, this, _1, _2, goal.first),
			boost::bind(&ROSControlPlayer::trajectoryClientHandleActive, this, goal.first)
		);
	}
}

void ROSControlPlayer::playMotionServerReportResult(const PlayResult& result, PlayMotionStatus status)
{
	m_playMotionResult = result;
	m_playMotionStatus = status;

	// Report result
	switch(m_playMotionStatus)
	{
		case PlayMotionStatus::PLAYING:
		case PlayMotionStatus::PREEMPTING:
			return; // no change
		case PlayMotionStatus::SUCCEEDED:
			m_playMotionGoal.setSucceeded(m_playMotionResult);
			break;
		case PlayMotionStatus::ABORTED:
			m_playMotionGoal.setAborted(m_playMotionResult);
			break;
		case PlayMotionStatus::PREEMPTED:
			m_playMotionGoal.setCanceled(m_playMotionResult);
			break;
	}

	// finished!
	m_playMotionGoal = PlayGoalHandle();
	playMotionServerSwitchGoal();
}

void ROSControlPlayer::playMotionServerCancelGoal(PlayGoalHandle handle)
{
	if(handle == m_playMotionGoal)
	{
		ROS_INFO("The currently active goal was canceled, cancelling all client goals");
		m_playMotionStatus = PlayMotionStatus::PREEMPTING;
		trajectoryClientsCancelAllGoals();
	}
	else if(handle == m_playMotionNextGoal)
	{
		ROS_INFO("The next goal was canceled");
		m_playMotionNextGoal.setCanceled(PlayResult(), "cancelled by client request");
		m_playMotionNextGoal = PlayGoalHandle();
	}
	else
	{
		ROS_INFO("Got cancel request for unrelated goal");
		handle.setCanceled(PlayResult(), "cancelled by client request");
	}
}

//@}


void ROSControlPlayer::modifyMotion(nimbro_keyframe_server::Motion::Ptr motion)
{
	// Note(sebastian): Implement this in a subclass!
}



/// Trajectory Server callbacks
//@{
void ROSControlPlayer::trajectoryServerHandleGoal()
{
	if(!m_act_trajectoryServer->isNewGoalAvailable())
		return;

	ROS_WARN("trajectory server is not implemented in a safe way yet!");

	const auto& goal = m_act_trajectoryServer->acceptNewGoal();

	if(m_playMotionGoal.isValid())
	{
		using namespace control_msgs;
		FollowJointTrajectoryResult result;
		result.error_code = FollowJointTrajectoryResult::INVALID_GOAL;
		result.error_string =
			"Play Motion Server still has active goal";
		m_act_trajectoryServer->setAborted(result);
		return;
	}

	splitTrajectory(goal->trajectory);

	for(auto& goal_pair : m_goals)
	{
		m_trajectoryClients[goal_pair.first]->sendGoal(
			goal_pair.second,
			boost::bind(&ROSControlPlayer::trajectoryClientHandleFinished,
				    this, _1, _2, goal_pair.first)
		);
	}
}

void ROSControlPlayer::trajectoryServerCancelGoal()
{
	ROS_INFO("Trajectory Server Goal is cancelled, so cancel all Client Goals");
	m_playMotionStatus = PlayMotionStatus::PREEMPTING;
	trajectoryClientsCancelAllGoals();
}

void ROSControlPlayer::abortTrajectoryGoal(uint8_t code, const std::string& error)
{
	trajectoryClientsCancelAllGoals();

	// Give negative result
	PlayResult new_result;

	new_result.finish_state = PlayResult::ABORTED_DURING_PLAYBACK;
	new_result.error_code = code;
	new_result.error_string = error;

	playMotionServerReportResult(new_result, PlayMotionStatus::ABORTED);
}
//@}


/// Trajectory Client callbacks
//@{
void ROSControlPlayer::trajectoryClientHandleProblem(
	const std::string& action,
	const actionlib::SimpleClientGoalState& state,
	const control_msgs::FollowJointTrajectoryResultConstPtr& result)
{
	ROS_INFO("Problem on one client, canceling all goals");

	// Cancel all other action clients
	trajectoryClientsCancelAllGoals();
	// If the action was already canceled or we already aborted it, do nothing more.
	if(m_playMotionStatus != PlayMotionStatus::PLAYING)
		return;

	// Give negative result
	PlayResult new_result;

	if(state == state.REJECTED)
		new_result.finish_state = PlayResult::REJECTED;
	else
		new_result.finish_state = PlayResult::ABORTED_DURING_PLAYBACK;

	if(!result->error_string.empty())
		new_result.error_string = result->error_string;
	else
		new_result.error_string = "Unknown error";

	new_result.failed_action = action;

	switch(result->error_code)
	{
		case control_msgs::FollowJointTrajectoryResult::INVALID_GOAL:
			new_result.error_code = PlayResult::INVALID_GOAL;
			break;
		case control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS:
			new_result.error_code = PlayResult::INVALID_JOINTS;
			break;
		case control_msgs::FollowJointTrajectoryResult::OLD_HEADER_TIMESTAMP:
			new_result.error_code = PlayResult::TIMEOUT;
			break;
		case control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED:
		case control_msgs::FollowJointTrajectoryResult::GOAL_TOLERANCE_VIOLATED:
			new_result.error_code = PlayResult::COLLISION;
			break;


		case control_msgs::FollowJointTrajectoryResult::SUCCESSFUL:
			// HACK: ur_modern_driver reports SUCCESSFUL, "Robot was halted" on emergency stop
			if(result->error_string == "EMERGENCY STOP")
			{
				new_result.error_code = PlayResult::EMERGENCYSTOP;
				break;
			}
			else if(result->error_string == "PROTECTIVE STOP")
			{
				new_result.error_code = PlayResult::PROTECTIVE_STOP;
				break;
			}

			// intended fallthrough, success on abort is still an error
		default:
			ROS_ERROR("Unknown FollowJointTrajectory result code: '%d' (%s), reporting TIMEOUT...",
				result->error_code, result->error_string.c_str()
			);
			new_result.error_code = PlayResult::TIMEOUT;
			break;
	}

	playMotionServerReportResult(new_result, PlayMotionStatus::ABORTED);
}


void ROSControlPlayer::trajectoryClientHandleFinished(const actionlib::SimpleClientGoalState& state,
		const control_msgs::FollowJointTrajectoryResultConstPtr& result, const std::string& actionName)
{
	if(!state.isDone())
		return;

	ROS_INFO("%s: got state '%s'", actionName.c_str(), state.toString().c_str());
	ROS_INFO("%s: current result: %s (%d)", actionName.c_str(), result->error_string.c_str(), result->error_code);

	if(m_playMotionStatus == PlayMotionStatus::PREEMPTING)
	{
		ROS_INFO("Preempt requested, waiting for action clients to confirm...");

		// We are currently canceling, so wait until all action clients have also canceled
		for(auto& goal : m_goals)
		{
			if(!m_trajectoryClients[goal.first]->getState().isDone())
			{
				ROS_INFO("%s is still running, waiting...", goal.first.c_str());
				return;
			}
		}

		ROS_INFO("All clients confirmed cancellation, confirming ours...");

		PlayResult result;
		result.success = false;
		result.error_code = -100;

		playMotionServerReportResult(result, PlayMotionStatus::PREEMPTED);
		return;
	}

	// Check if an action client has encountered an error
	if(state != state.SUCCEEDED || result->error_code != control_msgs::FollowJointTrajectoryResult::SUCCESSFUL)
	{
		trajectoryClientHandleProblem(actionName, state, result);
		return;
	}

	// Check if *all* action clients have finished
	for(auto& goal : m_goals)
	{
		if(!m_trajectoryClients[goal.first]->getState().isDone())
		{
			ROS_INFO("%s has not succeeded yet", goal.first.c_str());
			return;
		}
	}

	PlayResult new_result;
	new_result.success = true;
	new_result.error_code = 0;
	new_result.finish_state = PlayResult::FINISHED;
	playMotionServerReportResult(new_result, PlayMotionStatus::SUCCEEDED);
}

void ROSControlPlayer::trajectoryClientHandleActive(std::string actionName)
{
	ROS_DEBUG("Motion started for action: '%s'.", actionName.c_str());
	m_motion_started = ros::Time::now();
}

void ROSControlPlayer::trajectoryClientsCancelAllGoals()
{
	for(auto& client : m_trajectoryClients)
	{
		if(!client.second->getState().isDone())
			client.second->cancelAllGoals();
	}
}
//@}


/// Deprecated moveit player service callbacks
//@{
void ROSControlPlayer::playServiceHandleActionFinished(const std::string& name, const actionlib::SimpleClientGoalState& state,
		const control_msgs::FollowJointTrajectoryResultConstPtr& result)
{
	ROS_INFO("callback %s: %s ", name.c_str(), state.toString().c_str() );
}

bool ROSControlPlayer::play(const nimbro_keyframe_server::Motion::Ptr& motion)
{
	ROS_WARN("Triggering a motion via service call is deprecated.");
	ROS_WARN("Please use the action interface of the ROSControlPlayer!");
	{
		std::unique_lock<std::mutex> lock(m_robotStateMutex);
		*m_playerState = *m_robotState;
	}

	trajectory_score::TrajectoryScorer s(m_playerState, m_robotModel);
	auto score = s.scoreMotion(motion);

	if(score.inCollision)
	{
		ROS_ERROR("Collision in trajectory");
		return false;
	}

	for(auto& goal_pair: m_goals)
	{
		ROS_INFO("goal for %s", goal_pair.first.c_str());
		if(goal_pair.second.trajectory.points.size()>0)
		{
			m_trajectoryClients[goal_pair.first]->sendGoal(goal_pair.second, boost::bind(&ROSControlPlayer::playServiceHandleActionFinished, this, goal_pair.first, _1, _2));
		}
		else
		{
			ROS_ERROR("Empty trajectory for actionController '%s'",goal_pair.first.c_str());
		}
	}


	return true;
}
//@}

}
