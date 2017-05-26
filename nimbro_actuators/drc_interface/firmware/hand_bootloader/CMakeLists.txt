

set(CMAKE_C_COMPILER avr-gcc)
set(CMAKE_CXX_COMPILER avr-g++)

# Jump tables: The bootloader resides in the upper 64k of the flash, which
# gcc's jump table generator code cannot handle :-(
set(CMAKE_C_FLAGS "-Wall -Os -g -fno-exceptions -fno-jump-tables -mmcu=atmega128")
set(CMAKE_C_FLAGS_RELWITHDEBINFO "")
set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_definitions(-DF_CPU=16000000UL)

set(SRCS
	../dxlpro_slave/dxlpro_slave.c
	bootloader.c
)

add_executable(hand_bootloader
	${SRCS}
)

# Bootloader starts at 0xF000 (word address, largest bootloader size)
set_target_properties(hand_bootloader PROPERTIES
	LINK_FLAGS "-Wl,--section-start=.text=0x1E000"
)

add_custom_command(
	OUTPUT hand_bootloader.hex
	DEPENDS hand_bootloader
	COMMAND avr-objcopy -O ihex $<TARGET_FILE:hand_bootloader> ${CMAKE_CURRENT_BINARY_DIR}/hand_bootloader.hex
	VERBATIM
)
add_custom_target(hand_bootloader_hex ALL
	DEPENDS hand_bootloader.hex
)
