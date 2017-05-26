// APC control state base class
// Author: Christian Lenz <chrislenz@uni-bonn.de>

#ifndef APC_CONTROL_STATE_H
#define APC_CONTROL_STATE_H

#include <nimbro_fsm/fsm_ros.h>
#include "../control.h"

namespace apc_control
{

class State : public nimbro_fsm::StateROS<Control>
{
public:
	State();
	virtual ~State();

	virtual void exit() override;

};

}

#endif
