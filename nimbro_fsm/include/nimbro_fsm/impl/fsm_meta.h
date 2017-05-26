// Some template magic to get meta information about registered State classes
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef NIMBRO_FSM_IMPL_FSM_META_H
#define NIMBRO_FSM_IMPL_FSM_META_H

#include <typeinfo>

#include "utils.h"

namespace nimbro_fsm
{

class StateBase;

namespace meta
{

class StateMetaBase
{
public:
	StateMetaBase(const std::string& name)
	 : m_name(name)
	{}

	virtual StateBase* create() = 0;

	inline const std::string& name() const
	{ return m_name; }
private:
	std::string m_name;
};

template<class StateType>
class StateMeta : public StateMetaBase
{
public:
	StateMeta()
	 : StateMetaBase(impl::makePrettyStateName(typeid(StateType)))
	{}

	StateBase* create()
	{ return new StateType; }
};

}

}

#endif
