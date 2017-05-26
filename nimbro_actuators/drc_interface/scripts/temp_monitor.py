#!/usr/bin/python

import rospy
from momaro_interface.msg import Diagnostics
import os

lastBeepTime = None

def callback(msg):
	global lastBeepTime

	alert = False
	for joint in msg.joints:
		if joint.temperature >= 71:
			alert = True

	now = rospy.Time.now()
	if alert and (now - lastBeepTime) > rospy.Duration(4.0):
		os.system('beep -f 2000 -l 500')
		lastBeepTime = now

rospy.init_node('temp_monitor')

lastBeepTime = rospy.Time.now()

rospy.Subscriber('/momaro/diagnostics', Diagnostics, callback)

rospy.spin()

