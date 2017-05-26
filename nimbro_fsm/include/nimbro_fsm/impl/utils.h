// Utility functions
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef NIMBRO_FSM_IMPL_UTILS_H
#define NIMBRO_FSM_IMPL_UTILS_H

#include <typeinfo>
#include <string>

namespace nimbro_fsm
{

namespace impl
{

std::string makePrettyStateName(const std::type_info& info);

}

}

#endif
