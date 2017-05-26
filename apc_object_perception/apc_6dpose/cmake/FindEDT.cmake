#
# The following variables are optionally searched for defaults
#
# The following are set after configuration is done:
#  EDT_FOUND
#  EDT_INCLUDE_DIRS


FIND_PATH( EDT_INCLUDE_DIR NAMES "EDT/distance_field.h"
	      PATHS /usr/include $ENV{EDT_DIR} ${EDT_DIR}
	      PATH_SUFFIXES EDT)


IF (EDT_INCLUDE_DIR)
   SET(EDT_FOUND TRUE)
   MESSAGE(STATUS "Found EDT")
ELSE (EDT_INCLUDE_DIR)
   SET( EDT_FOUND FALSE )
   MESSAGE(FATAL_ERROR "Unable to find EDT")
ENDIF (EDT_INCLUDE_DIR )


if(EDT_FOUND)
  set(EDT_INCLUDE_DIRS ${EDT_INCLUDE_DIR})
endif(EDT_FOUND)

