#!/bin/sh
find . -name frame.pcd | while read fname; 
do 
	pcl_pcd2png --field z $fname "${fname%/*}/depth.png"; 
done
