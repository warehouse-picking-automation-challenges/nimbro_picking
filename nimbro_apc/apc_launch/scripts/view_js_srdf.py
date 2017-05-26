#!/usr/bin/python

import rospy
from sensor_msgs.msg import JointState

js = None
diag = None

def recv_js(new_js):
        global js
	if len(new_js.name) < 5:
		return
	js = new_js

def print_info(ev):
	if js is None:
		print "No info yet"
		return

	print
	print "==========================================="
	for name, pos, effort in zip(js.name, js.position, js.effort):
		print "<joint name=\"%s\" value=\"%f\" />" % (name, pos)

rospy.init_node('view_js')
sub_js = rospy.Subscriber("/joint_states", JointState, recv_js)
timer = rospy.Timer(rospy.Duration(0.5), print_info)

rospy.spin()


