
cmake_minimum_required(VERSION 2.8)
project(reflexxes)

find_package(catkin REQUIRED)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES reflexxes
)

include_directories(include/reflexxes)

add_library(reflexxes
	src/TypeIIRML/ReflexxesAPI.cpp
	src/TypeIIRML/TypeIIRMLCalculatePositionalExtrems.cpp
	src/TypeIIRML/TypeIIRMLDecisions.cpp
	src/TypeIIRML/TypeIIRMLDecisionTree1A.cpp
	src/TypeIIRML/TypeIIRMLDecisionTree1B.cpp
	src/TypeIIRML/TypeIIRMLDecisionTree1C.cpp
	src/TypeIIRML/TypeIIRMLDecisionTree2.cpp
	src/TypeIIRML/TypeIIRMLFallBackStrategy.cpp
	src/TypeIIRML/TypeIIRMLIsPhaseSynchronizationPossible.cpp
	src/TypeIIRML/TypeIIRMLPolynomial.cpp
	src/TypeIIRML/TypeIIRMLPosition.cpp
	src/TypeIIRML/TypeIIRMLQuicksort.cpp
	src/TypeIIRML/TypeIIRMLSetupModifiedSelectionVector.cpp
	src/TypeIIRML/TypeIIRMLStep1.cpp
	src/TypeIIRML/TypeIIRMLStep1IntermediateProfiles.cpp
	src/TypeIIRML/TypeIIRMLStep1Profiles.cpp
	src/TypeIIRML/TypeIIRMLStep2.cpp
	src/TypeIIRML/TypeIIRMLStep2IntermediateProfiles.cpp
	src/TypeIIRML/TypeIIRMLStep2PhaseSynchronization.cpp
	src/TypeIIRML/TypeIIRMLStep2Profiles.cpp
	src/TypeIIRML/TypeIIRMLStep2WithoutSynchronization.cpp
	src/TypeIIRML/TypeIIRMLStep3.cpp
	src/TypeIIRML/TypeIIRMLVelocityCalculatePositionalExtrems.cpp
	src/TypeIIRML/TypeIIRMLVelocity.cpp
	src/TypeIIRML/TypeIIRMLVelocityFallBackStrategy.cpp
	src/TypeIIRML/TypeIIRMLVelocityIsPhaseSynchronizationPossible.cpp
	src/TypeIIRML/TypeIIRMLVelocityMethods.cpp
	src/TypeIIRML/TypeIIRMLVelocitySetupPhaseSyncSelectionVector.cpp
)
