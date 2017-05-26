// Generic Finite State Machine
// Author: Jörg Stückler <stueckler@ais.uni-bonn.de>
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef NIMBRO_FSM_FSM_H
#define NIMBRO_FSM_FSM_H

#include <stdexcept>
#include <sstream>
#include <typeinfo>
#include <vector>

#include <boost/utility/enable_if.hpp>
#include <boost/circular_buffer.hpp>

#include <nimbro_fsm/impl/fsm_meta.h>

//! Generic Finite State Machine
namespace nimbro_fsm
{

class StateMachine;

/**
 * @brief Driver interface
 *
 * The nimbro_fsm library supports the notion of a "driver", a central object
 * all states get access to via their State::driver() method.
 *
 * Subclass this class to provide the states with means to interact with
 * the world. The instance is passed to the StateMachine constructor.
 **/
class Driver
{
public:
	virtual ~Driver();
};

/**
 * @brief State base class
 *
 * All states must be derived from this class. You will have to implement at
 * least the execute() function.
 *
 * @warning Contrary to earlier NimbRo FSM designs, the @b constructor is not
 *   called at state entry time. This means that the driver() is not available
 *   in the constructor. For work that should be done on state entry, please
 *   override the enter() method.
 *
 * @note For nicer access to your sub-classed Driver object, see DriverState.
 **/
class StateBase
{
friend class StateMachine;
public:
	StateBase();
	virtual ~StateBase();

	/**
	 * @brief State entry hook
	 *
	 * This method is called on state entry.
	 *
	 * @warning If you override this method, take care to call State::entry()
	 *   at the top of your implementation.
	 **/
	virtual void enter();

	/**
	 * @brief State execute hook
	 *
	 * This method is called whenever StateMachine::execute() is called. Use it
	 * to specify work to be done in each state machine iteration.
	 *
	 * The return value is used to decide the next state machine state. You can:
	 *
	 * * Return @b this to stay in the current state
	 * * Return a new StateBase instance to switch state
	 * * Return 0 to halt the StateMachine (see StateMachine::isFinished())
	 **/
	virtual StateBase* execute() = 0;

	/**
	 * @brief State exit hook
	 *
	 * This method is called on state exit.
	 *
	 * @warning If you override this method, take care to call State::entry()
	 *   at the top of your implementation.
	 **/
	virtual void exit();

	/**
	 * @brief Driver access
	 *
	 * @return Pointer to the Driver object associated with the FSM.
	 *
	 * See DriverState for a more comfortable access to the driver object.
	 **/
	inline Driver* driver()
	{ return m_driver; }

	/**
	 * @brief Driver access
	 *
	 * @return Pointer to the Driver object associated with the FSM.
	 *
	 * See DriverState for a more comfortable access to the driver object.
	 **/
	inline const Driver* driver() const
	{ return m_driver; }
private:
	Driver* m_driver;
};

/**
 * @brief Convenience State base class
 *
 * This template class provides convenience driver() access for your specific
 * Driver subclass.
 *
 * Suggested usage:
 *
 * @code
 * class MyDriver {....};
 * typedef nimbro_fsm::DriverState<MyDriver> State;
 *
 * class StateA : public State {...};
 * @endcode
 **/
template<class DriverType>
class DriverState : public StateBase
{
public:
	DriverState() {}
	virtual ~DriverState() {}

	virtual void enter()
	{
		if(!dynamic_cast<DriverType*>(StateBase::driver()))
		{
			std::stringstream ss;
			ss << "nimbro_fsm::DriverState: Got invalid driver, expected "
			   << typeid(DriverType).name();
			throw std::runtime_error(ss.str());
		}
	}

	/**
	 * @brief Driver access
	 *
	 * @return Pointer to the Driver object associated with the FSM.
	 **/
	inline DriverType* driver()
	{ return (DriverType*)(StateBase::driver()); }

	/**
	 * @brief Driver access
	 *
	 * @return Pointer to the Driver object associated with the FSM.
	 **/
	inline const DriverType* driver() const
	{ return (const DriverType*)(StateBase::driver()); }
};

/**
 * @brief State Machine class
 *
 * Represents a finite state machine (FSM).
 **/
class StateMachine
{
public:
	/**
	 * @brief Constructor
	 *
	 * @param driver Driver instance to be associated with the FSM (may be 0)
	 **/
	explicit StateMachine(Driver* driver);
	virtual ~StateMachine();

	/**
	 * @brief Current state
	 *
	 * @return pointer to current state, may be zero if isFinished().
	 **/
	inline StateBase* state()
	{ return m_state; }

	/**
	 * @brief Current state
	 *
	 * @return pointer to current state, may be zero if isFinished().
	 **/
	inline const StateBase* state() const
	{ return m_state; }

	/**
	 * @brief Current state name
	 *
	 * This uses C++ RTTI to determine the name of the current state.
	 *
	 * @return Name of the current state.
	 **/
	inline const std::string& stateName() const
	{ return m_stateName; }

	/**
	 * @brief Force a state change
	 *
	 * This method forces an immediate state change to state @b state.
	 **/
	virtual void changeState( StateBase* state );

	/**
	 * @brief FSM iteration
	 *
	 * This method advances the FSM by one step. The current state's
	 * State::execute() hook is called and appropriate action is taken.
	 **/
	virtual void execute();

	/**
	 * @brief Register state
	 *
	 * State registration is @b not necessary for normal operation. It provides
	 * the FSM with information on how to @b create a state instance of the
	 * registered type by itself. This may be necessary if the StateMachineROS
	 * subclass is used.
	 **/
	template<class StateType>
	void registerState();

	/**
	 * @brief Pause the state machine execution
	 *
	 * @param paused If true, the execute() method will return without doing
	 *               anything.
	 **/
	void setPaused(bool paused);

	//! Is the FSM paused?
	inline bool paused() const
	{ return m_paused; }

	/**
	 * @brief Check if finished
	 *
	 * The FSM is @b finished if a state's State::execute() hook returns 0.
	 *
	 * @note The FSM starts up with finished() == true.
	 **/
	inline bool finished() const
	{ return !m_state; }
protected:
	virtual void setupState(StateBase* state);
	virtual void onFinished();

	void registerState(meta::StateMetaBase* meta);

	Driver* m_driver;

	StateBase* m_state;
	std::string m_stateName;

	boost::circular_buffer<std::string> m_stateHistory;

	std::vector<meta::StateMetaBase*> m_registeredStates;

	bool m_paused;
};

template<class StateType>
void StateMachine::registerState()
{
	registerState(new meta::StateMeta<StateType>());
}

}

#endif
