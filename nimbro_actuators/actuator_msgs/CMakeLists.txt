
cmake_minimum_required(VERSION 2.8)
project(actuator_msgs)

find_package(catkin REQUIRED COMPONENTS
	message_generation
	std_msgs
)

add_message_files(FILES
	Diagnostics.msg
	JointDiagnostics.msg
	JointTemperature.msg
	JointTemperatures.msg
)

add_service_files(FILES
	FadeJoints.srv
	RebootServo.srv
)

generate_messages(DEPENDENCIES
	std_msgs
)

catkin_package()
