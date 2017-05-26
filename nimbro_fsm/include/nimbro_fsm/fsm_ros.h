// FSM with ROS diagnostics
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef NIMBRO_FSM_FSM_ROS_H
#define NIMBRO_FSM_FSM_ROS_H

#include "fsm.h"

#include <ros/node_handle.h>

#include <nimbro_fsm/ChangeState.h>
#include <nimbro_fsm/Status.h>

namespace nimbro_fsm
{

/**
 * @brief Convenience base class for ROS states
 **/
template<class DriverType>
class StateROS : public DriverState<DriverType>
{
public:
	/**
	 * @brief Elapsed time
	 *
	 * @return Elapsed time since enter() was called.
	 **/
	inline ros::Duration elapsedTime() const
	{ return ros::Time::now() - m_startTime; }

	virtual void enter()
	{ m_startTime = ros::Time::now(); }
private:
	ros::Time m_startTime;
};

/**
 * @brief State Machine class for ROS-based FSMs
 *
 * This class exports a ROS interface that can be used to visualize the FSM's
 * state and change it via service calls.
 *
 * Note that changing into states via the service call is only supported if
 * they have been registered using the StateMachine::registerState() method.
 **/
class StateMachineROS : public StateMachine
{
public:
	/**
	 * @brief Constructor
	 *
	 * @param nh NodeHandle to create the ROS interface under. The FSM topics
	 *           and services will be called fsm/XYZ.
	 * @param driver Central driver object (see StateMachine::StateMachine()).
	 **/
	explicit StateMachineROS(ros::NodeHandle nh, Driver* driver);
	virtual ~StateMachineROS();

	virtual void execute();
	virtual void changeState(StateBase* state);
protected:
	virtual void setupState(StateBase* state);
	virtual void onFinished();
private:
	bool changeStateRequest(ChangeStateRequest& req, ChangeStateResponse&);

	bool m_info_published;
	ros::Publisher m_pub_info;
	ros::Publisher m_pub_status;
	ros::ServiceServer m_srv_switchState;

	nimbro_fsm::Status m_statusMsg;
};

}

#endif
