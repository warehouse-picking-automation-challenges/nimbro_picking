// Utility functions
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <nimbro_fsm/impl/utils.h>

// Include cxxabi.h for __cxa_demangle is possible
#if (defined(__GNUC__)                                                  \
   && ((__GNUC__ > 3) || ((__GNUC__ == 3) && (__GNUC_MINOR__ >= 1)))   \
   && !defined(__EDG_VERSION__)) || defined(__clang__)
#define HAVE_DEMANGLE 1
#include <cxxabi.h>
#else
#define HAVE_DEMANGLE 0
#endif

namespace nimbro_fsm
{

namespace impl
{

std::string makePrettyStateName(const std::type_info& info)
{
	const char* rawName = info.name();

#if HAVE_DEMANGLE
	int status;
	std::string clsName = abi::__cxa_demangle(rawName, 0, 0, &status);
#else
	std::string clsName = rawName;
#endif

	// If the result contains a namespace, strip it (only the top one)
	int pos = clsName.find("::");
	if(pos >= 0)
		clsName = clsName.substr(pos+2);

	return clsName;
}

}

}
