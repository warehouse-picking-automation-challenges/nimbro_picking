
set(CMAKE_C_COMPILER avr-gcc)
set(CMAKE_CXX_COMPILER avr-g++)

set(CMAKE_CXX_FLAGS "-g -fno-exceptions -mmcu=atmega2560 -Wall")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O2")
set(CMAKE_CXX_FLAGS_DEBUG "-O2") # AVR code should always be optimized - see avr-libc / delay.h
set(CMAKE_C_FLAGS "-g -mmcu=atmega2560 -Wall")
set(CMAKE_C_FLAGS_RELWITHDEBINFO "-O2")
set(CMAKE_C_FLAGS_DEBUG "-O2")
set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS)

add_definitions(-DF_CPU=16000000UL)

find_package(libucomm HINTS /usr/lib/avr REQUIRED)
include_directories(${LIBUCOMM_INCLUDE_DIRS})

include_directories(.)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

libucomm_wrap_msg(
	MSG_SRC
	../protocol/apc_proto.msg
)

add_executable(apc_controller
	${MSG_SRC}
	adc.cpp
	io.cpp
	dxl_master/dxl_master.c
	dxl.cpp
	comm.cpp
	uart.c
	main.cpp
)

get_target_property(location apc_controller LOCATION)

add_custom_command(
	OUTPUT apc_controller.hex
	DEPENDS apc_controller
	COMMAND avr-objcopy -O ihex ${location} apc_controller.hex
)

add_custom_target(writeflash
	DEPENDS apc_controller.hex
	COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/writeflash.sh apc_controller.hex
)

