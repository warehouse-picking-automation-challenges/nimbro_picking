#!/bin/bash

if [[ ! $# -eq 2 ]]; then
	echo "Usage: capture_data.sh bin_x bin_y"
	exit 1
fi

rosrun apc_capture apc_capture __name:=capture _save_path:=/tmp/capture_$(date +"%Y%m%d_%H%M") /capture/cloud:=/camera_filler/output /capture/camera_info:=/camera_stereo/cam2/camera_info _bin_x:=$1 _bin_y:=$2

