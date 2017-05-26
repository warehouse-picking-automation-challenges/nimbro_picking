// Generic Finite State Machine
// Author: Jörg Stückler <stueckler@ais.uni-bonn.de>
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <nimbro_fsm/fsm.h>

namespace nimbro_fsm
{

Driver::~Driver()
{
}

StateBase::StateBase()
{
}

StateBase::~StateBase()
{
}

void StateBase::enter()
{
}

void StateBase::exit()
{
}

StateMachine::StateMachine(Driver* driver)
 : m_driver(driver)
 , m_state(0)
 , m_stateHistory(5)
 , m_paused(false)
{
}

StateMachine::~StateMachine()
{
	if(m_state)
	{
		m_state->exit();
		delete m_state;
	}
}

void StateMachine::changeState(StateBase* state)
{
	if(m_state)
	{
		m_stateHistory.push_back(m_stateName);

		m_state->exit();
		delete m_state;
	}

	if(state)
	{
		m_state = state;
		m_stateName = impl::makePrettyStateName(typeid(*m_state));
		setupState(m_state);
		m_state->enter();
	}
	else
	{
		m_state = 0;
		m_stateName = "<finished>";
		onFinished();
	}
}

void StateMachine::setPaused(bool paused)
{
	m_paused = paused;
}

void StateMachine::execute()
{
	if(!m_state || m_paused)
		return;

	StateBase* newState = m_state->execute();

	if(newState != m_state)
		changeState(newState);
}

void StateMachine::setupState(StateBase* state)
{
	state->m_driver = m_driver;
}

void StateMachine::registerState(meta::StateMetaBase* meta)
{
	m_registeredStates.push_back(meta);
}

void StateMachine::onFinished()
{
}

}
