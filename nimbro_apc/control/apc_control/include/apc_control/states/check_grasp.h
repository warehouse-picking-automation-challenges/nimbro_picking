// Confirm that we have the object, otherwise press harder
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef CHECK_GRASP_STATE_H
#define CHECK_GRASP_STATE_H

#include <apc_control/states/state.h>

namespace apc_control
{

class CheckGrasp : public State
{
public:
	CheckGrasp();
	virtual ~CheckGrasp();

	virtual nimbro_fsm::StateBase* execute() override;
};

}

#endif
