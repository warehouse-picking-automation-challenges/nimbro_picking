
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

function(add_writeflash_target name)
	add_custom_command(
		OUTPUT ${name}.hex
		DEPENDS ${name}
		COMMAND avr-objcopy -O ihex $<TARGET_FILE:${name}> ${name}.hex
		VERBATIM
	)

	add_custom_target(writeflash_${name}_front_left
		DEPENDS ${name}.hex
		COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/writeflash.sh /dev/momaro/front_left ${name}.hex
	)
	add_custom_target(writeflash_${name}_front_right
		DEPENDS ${name}.hex
		COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/writeflash.sh /dev/momaro/front_right ${name}.hex
	)
	add_custom_target(writeflash_${name}_back_left
		DEPENDS ${name}.hex
		COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/writeflash.sh /dev/momaro/back_left ${name}.hex
	)
	add_custom_target(writeflash_${name}_back_right
		DEPENDS ${name}.hex
		COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/writeflash.sh /dev/momaro/back_right ${name}.hex
	)
	add_custom_target(writeflash_${name}_left_arm
		DEPENDS ${name}.hex
		COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/writeflash.sh /dev/momaro/left_arm ${name}.hex
	)
	add_custom_target(writeflash_${name}_right_arm
		DEPENDS ${name}.hex
		COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/writeflash.sh /dev/momaro/right_arm ${name}.hex
	)
	add_custom_target(generate_hex
		DEPENDS ${name}.hex
	)

	add_custom_target(writeflash_all
		DEPENDS writeflash_${name}_left_arm writeflash_${name}_right_arm writeflash_${name}_front_left writeflash_${name}_front_right writeflash_${name}_back_right writeflash_${name}_back_left
	)
endfunction()

find_package(libucomm HINTS /usr/lib/avr REQUIRED)
include_directories(${LIBUCOMM_INCLUDE_DIRS})

include_directories(${CMAKE_CURRENT_BINARY_DIR})

libucomm_wrap_msg(
	MSG_SRC
	../../protocol/drc_proto.msg
)

add_executable(drc_controller
	${MSG_SRC}
	main.cpp
	uart.c
	comm_pc.cpp
	comm_dxl.cpp
	servo.cpp
	io.cpp
	imu.cpp
)
add_writeflash_target(drc_controller)
