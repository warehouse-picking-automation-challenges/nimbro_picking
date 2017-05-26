#
# The following variables are optionally searched for defaults
#
# The following are set after configuration is done:
#  MLK_FOUND
#  MLK_INCLUDE_DIRS
#  MLK_LIBRARIES

#message(STATUS "Looking for file mkl_cblas.h")



FIND_PATH( MLK_INCLUDE_DIR NAMES mkl_cblas.h
	      PATHS /opt/intel/mkl/include)

FIND_PATH( MLK_INCLUDE NAMES mkl_cblas.h
              PATHS /opt/intel/mkl/include)

#IF (MKL_INCLUDE_DIR)
#  list ( APPEND MKL_INCLUDE_DIRS MKL_INCLUDE_DIR)
#ENDIF(MKL_INCLUDE_DIR)


#find_library(MLK_LIBRARY NAMES libmkl_intel_lp64 libmkl_core libmkl_gnu_thread libdl libpthread libm HINTS /opt/intel/mkl /opt/intel/mkl/lib /opt/intel/mkl/lib/intel64 /usr/lib/x86_64-linux-gnu)

#find_library(MLK_LIBRARY NAMES libmkl_intel_lp64.a libmkl_intel_lp64.so 
#				libmkl_core.a libmkl_core.so
#				libmkl_gnu_thread.a libmkl_gnu_thread.so
#				libdl.so libdl.a
#				libpthread.a libpthread.so
#				libm.a libm.so
find_library(MLK_LIBRARY NAMES mkl_intel_lp64 mkl_core mkl_gnu_thread
			PATHS /opt/intel/mkl /opt/intel/mkl/lib /opt/intel/mkl/lib/intel64 /usr/lib/x86_64-linux-gnu)


#include(FindPackageHandleStandardArgs)
#find_package_handle_standard_args(MKL DEFAULT_MSG MKL_INCLUDE_DIR MKL_LIBRARY)

IF (MKL_LIBRARY)			
  list ( APPEND MKL_LIBRARY mkl_intel_lp64)
  list ( APPEND MKL_LIBRARY mkl_core)
  list ( APPEND MKL_LIBRARY mkl_gnu_thread)
  list ( APPEND MKL_LIBRARY dl)
  list ( APPEND MKL_LIBRARY pthread)
  list ( APPEND MKL_LIBRARY m)
ENDIF(MKL_LIBRARY)

#message(STATUS "Found libmkl_intel_lp64 libmkl_core libmkl_gnu_thread libdl libpthread libm " ${MLK_LIBRARY})


#include(FindPackageHandleStandardArgs)
#find_package_handle_standard_args(MKL DEFAULT_MSG LIB_MKL)


IF (MLK_INCLUDE_DIR AND MLK_LIBRARY)
   SET(MKL_FOUND TRUE)
   MESSAGE(STATUS "Found MKL")
ELSE (MLK_INCLUDE_DIR AND MLK_LIBRARY)
   SET( MKL_FOUND FALSE )
   MESSAGE(FATAL_ERROR "Unable to find MKL")
ENDIF (MLK_INCLUDE_DIR AND MLK_LIBRARY)


#set(MKL_PROCESS_INCLUDES MKL_INCLUDE_DIR MKL_INCLUDE_DIRS)
#set(MKL_PROCESS_LIBS MKL_LIBRARY MKL_LIBRARIES)
#set(MKL_INCLUDE_DIRS ${MKL_INCLUDE_DIR})
#set(MKL_LIBRARIES ${MKL_LIBRARY})

#libfind_process(MKL)

if(MKL_FOUND)
  set(MKL_LIBRARIES ${MKL_LIBRARY})
  set(MKL_INCLUDE_DIRS ${MLK_INCLUDE_DIR})
#  message(STATUS "Found MKL (library: ${MKL_LIBRARY})")
endif(MKL_FOUND)

