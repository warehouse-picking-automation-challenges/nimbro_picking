#!/usr/bin/python

import rospy
from sensor_msgs.msg import JointState

g_state = {}

js = None
diag = None

def recv_js(new_js):
        global g_state

	for name, pos, effort in zip(new_js.name, new_js.position, new_js.effort):
		g_state[name] = (pos, effort)

def print_info(ev):
	print
	print "==========================================="
	for name, state in sorted(g_state.items()):
		print "%30s: % 10.3f (effort: % 10.3f)" % (name, state[0], state[1])

rospy.init_node('view_js')
sub_js = rospy.Subscriber("/joint_states", JointState, recv_js)
timer = rospy.Timer(rospy.Duration(0.1), print_info)

rospy.spin()
