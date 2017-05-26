// Move arm state for APC
// Author: Christian Lenz <chrislenz@uni-bonn.de>

#include <ros/console.h>
#include <apc_control/control.h>
#include <apc_control/states/move_arm.h>
#include <apc_control/states/reset.h>
#include <nimbro_keyframe_server/motion.h>

#include <ros/service.h>
#include <std_srvs/Empty.h>

namespace apc_control
{

	MoveArm::MoveArm()
	 : m_actionClient("/ros_player/play_motion")
	{
	}

	MoveArm::~MoveArm()
	{
	}

	void MoveArm::enter()
	{
		State::enter();

		if(!m_actionClient.waitForServer())
		{
			ROS_ERROR("Could not wait for action server");
			throw std::runtime_error("Could not wait for action server");
		}

		m_goal = computeGoal();

		m_actionClient.sendGoal(m_goal, ActionClient::SimpleDoneCallback(), ActionClient::SimpleActiveCallback(), boost::bind(&MoveArm::handleFeedback, this, _1));
		m_tries++;
	}

	void MoveArm::exit()
	{
		State::exit();

		if(m_actionClient.isServerConnected())
		{
			if(!m_actionClient.getState().isDone())
			{
				ROS_WARN("Canceling active motion goal...");
				m_actionClient.cancelGoal();
			}
		}
	}


	void MoveArm::handleFeedback(const nimbro_keyframe_server::PlayMotionFeedbackConstPtr& feedback)
	{
		std::unique_lock<std::mutex> lock(m_mutex);
		m_feedback = feedback;
	}

	nimbro_keyframe_server::PlayMotionFeedbackConstPtr MoveArm::feedback()
	{
		std::unique_lock<std::mutex> lock(m_mutex);
		return m_feedback;
	}



	nimbro_fsm::StateBase* MoveArm::execute()
	{
		const actionlib::SimpleClientGoalState& state = m_actionClient.getState();
		const nimbro_keyframe_server::PlayMotionResultConstPtr& result = m_actionClient.getResult();

		if(state.isDone())
		{
			if(result->success)
			{
				ROS_INFO("result: SUCCESS");
				return nextState();
			}
			else
			{
				if(result->error_code == nimbro_keyframe_server::PlayMotionResult::INVALID_GOAL)
				{
					// This error can happen if the player does not have the newest joint states yet.
					// Simply try again.

					if(m_tries < 4)
					{
						ROS_ERROR("Got the INVALID_GOAL error, trying again...");
						usleep(100 * 1000);
						m_actionClient.sendGoal(m_goal);
						m_tries++;
						return this;
					}
				}

				if(result->error_code == nimbro_keyframe_server::PlayMotionResult::PROTECTIVE_STOP)
				{
					ROS_ERROR("protective stop!");
					return onProtectiveStop(result);
				}

				// Check if the motion was aborted before playback
				if(result->finish_state == nimbro_keyframe_server::PlayMotionResult::REJECTED)
				{
					if(result->error_string == "Cannot accept new trajectories: Robot is protective stopped")
					{
						ROS_ERROR("Motion rejected b/c of protective stop.");
						return onProtectiveStop(result);
					}
					else if(result->error_code == nimbro_keyframe_server::PlayMotionResult::COLLISION)
					{
						ROS_ERROR("Motion rejected b/c of predicted collision");
						return onPredictedCollision();
					}
				}

				ROS_INFO("result: '%s' (%d)", driver()->report_error(result->error_code).c_str(), result->error_code);
				driver()->shutdown();
				return 0;
			}
		}

// 		switch(m_actionClient.getState().state_)
// 		{
// // 			ROS_INFO("teeest %s", m_actionClient.getState().getText());
// 			case actionlib::SimpleClientGoalState::SUCCEEDED:
// 				ROS_INFO("Success");
// 				return new StateB();//Wait(ros::Duration(4.0), m_nextState.release());
// 			case actionlib::SimpleClientGoalState::ABORTED:
// 				ROS_ERROR("Move arm action was ABORTED: %s", m_actionClient.getState().getText().c_str());
// // 				ROS_ERROR("Continuing with next task...");
// 				return 0;
// 			case actionlib::SimpleClientGoalState::PENDING:
// 			case actionlib::SimpleClientGoalState::ACTIVE:
// 				break;
// 			case actionlib::SimpleClientGoalState::RECALLED:
// 			case actionlib::SimpleClientGoalState::REJECTED:
// 			case actionlib::SimpleClientGoalState::PREEMPTED:
// 			case actionlib::SimpleClientGoalState::LOST:
// 			default:
// 				ROS_ERROR("Unexpected action state '%s', aborting mission", m_actionClient.getState().getText().c_str());
// // 				driver()->abortMission();
// 				return 0;
// 		}

		return this;
	}

	bool MoveArm::unlockProtectiveStop()
	{
		// Try to avoid UR race condition
		sleep(1);

		std_srvs::Empty srv;
		bool ret = ros::service::call("/ur_driver/unlock_protective_stop", srv);
		if(!ret)
		{
			ROS_ERROR("Could not unlock the protective stop");
		}

		return ret;
	}

	nimbro_fsm::StateBase* MoveArm::onProtectiveStop(const nimbro_keyframe_server::PlayMotionResultConstPtr& result)
	{
		// If the stop happened *before* this motion, it is actually likely that
		// the current motion leads out of the protective stop.

		if(m_tries < 2 && result->finish_state == nimbro_keyframe_server::PlayMotionResult::REJECTED)
		{
			ROS_ERROR("Protective stop happened before this motion, unlocking and trying again...");
			unlockProtectiveStop();

			sleep(1);

			m_actionClient.sendGoal(m_goal);
			m_tries++;

			return this;
		}

		ROS_ERROR("Ran into protective stop. I will unlock the robot and exit.");
		unlockProtectiveStop();

		sleep(3.0);

		return new Reset();
	}

	nimbro_fsm::StateBase* MoveArm::onPredictedCollision()
	{
		ROS_ERROR("Motion would collide. Stopping...");
// 		driver()->shutdown();
		return new Reset();
	}
}
