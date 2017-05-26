// FSM with ROS diagnostics
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <nimbro_fsm/fsm_ros.h>

#include <nimbro_fsm/impl/utils.h>

#include <nimbro_fsm/Info.h>
#include <nimbro_fsm/Status.h>

#include <boost/foreach.hpp>



namespace nimbro_fsm
{

StateMachineROS::StateMachineROS(ros::NodeHandle nh, Driver* driver)
 : StateMachine(driver)
 , m_info_published(false)
{
	m_pub_info = nh.advertise<nimbro_fsm::Info>("fsm/info", 1, true);
	m_pub_status = nh.advertise<nimbro_fsm::Status>("fsm/status", 1);
	m_srv_switchState = nh.advertiseService("fsm/change_state",
		&StateMachineROS::changeStateRequest, this
	);
}

StateMachineROS::~StateMachineROS()
{
}

void StateMachineROS::setupState(StateBase* state)
{
	StateMachine::setupState(state);

	ROS_INFO("nimbro_fsm: entering state '%s'", stateName().c_str());

	if(!m_info_published)
	{
		nimbro_fsm::Info info;
		BOOST_FOREACH(meta::StateMetaBase* stateMeta, m_registeredStates)
		{
			info.constructible_states.push_back(stateMeta->name());
		}

		m_pub_info.publish(info);
	}
}

void StateMachineROS::changeState(StateBase* state)
{
	StateMachine::changeState(state);

	m_statusMsg.current_state = m_stateName;

	m_statusMsg.state_history.resize(m_stateHistory.size());
	std::copy(m_stateHistory.begin(), m_stateHistory.end(),
		m_statusMsg.state_history.begin()
	);
}

void StateMachineROS::execute()
{
	StateMachine::execute();

	m_statusMsg.paused = paused();
	m_pub_status.publish(m_statusMsg);
}

bool StateMachineROS::changeStateRequest(ChangeStateRequest& req, ChangeStateResponse&)
{
	std::vector<meta::StateMetaBase*>::iterator it;
	for(it = m_registeredStates.begin(); it != m_registeredStates.end(); ++it)
	{
		if((*it)->name() == req.state)
			break;
	}

	if(it == m_registeredStates.end())
	{
		ROS_ERROR("Invalid state '%s' requested", req.state.c_str());
		return false;
	}

	changeState((*it)->create());
	return true;
}

void StateMachineROS::onFinished()
{
	StateMachine::onFinished();

	ROS_INFO("nimbro_fsm: State machine finished.");
}

}
