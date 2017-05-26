#!/bin/sh
find . -name frame.pcd | while read fname; 
do 
	pcl_convert_pcd_ascii_binary $fname "${fname%/*}/frame_uncompressed.pcd" 1;
done
