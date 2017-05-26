#!/bin/bash -e

file="$1"

if [[ ! -w "/dev/apc_controller" ]]; then
	echo "Waiting for serial device /dev/apc_controller to appear..."
	while [[ ! -w "/dev/apc_controller" ]]; do
		true
	done
else
	echo "Resetting controller..."
	rosrun apc_interface controller_tool --device=/dev/apc_controller --reset
fi

avrdude -c avr109 -p atmega2560 -b 115200 -P "/dev/apc_controller" -U flash:w:"$1"

