
find_path(SIMAVR_PATH simavr/sim/sim_avr.h HINTS /opt/simavr)

if(NOT SIMAVR_PATH)
	message(WARNING "simavr not found, not building firmware tests")
	return()
endif()

find_library(SIMAVR_LIBRARY simavr HINTS ${SIMAVR_PATH}/simavr/obj-x86_64-linux-gnu)

get_target_property(FIRMWARE_PATH drc_controller LOCATION)

add_definitions(-DFIRMWARE_BINARY_PATH="${FIRMWARE_PATH}")
include_directories(${SIMAVR_PATH})

catch_add_test(firmware_tests
	test_support.cpp
	test_comm_pc.cpp
)
target_link_libraries(firmware_tests
	${SIMAVR_LIBRARY}
)
