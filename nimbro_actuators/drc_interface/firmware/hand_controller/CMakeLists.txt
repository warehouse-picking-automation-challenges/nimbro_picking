
set(CMAKE_C_COMPILER avr-gcc)
set(CMAKE_CXX_COMPILER avr-g++)

set(CMAKE_C_FLAGS "-Wall -Os -fno-exceptions -mmcu=atmega128")
set(CMAKE_C_FLAGS_RELWITHDEBINFO "")
set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_definitions(-DF_CPU=16000000UL)

set(SRCS
	../dxlpro_slave/dxlpro_slave.c
	../dxl_master/dxl_master.c
	main.c
	comm.c
	io.c
)

add_executable(hand_controller
	${SRCS}
)

add_custom_command(
	OUTPUT hand_controller.hex
	DEPENDS hand_controller
	COMMAND avr-objcopy -O ihex $<TARGET_FILE:hand_controller> ${CMAKE_CURRENT_BINARY_DIR}/hand_controller.hex
	VERBATIM
)
add_custom_target(hand_controller_hex ALL
	DEPENDS hand_controller.hex
)

add_custom_command(
	OUTPUT hand_controller.bin
	DEPENDS hand_controller
	COMMAND avr-objcopy -O binary $<TARGET_FILE:hand_controller> ${CMAKE_CURRENT_BINARY_DIR}/hand_controller.bin
)
add_custom_target(hand_controller_bin ALL
	DEPENDS hand_controller.bin
)
