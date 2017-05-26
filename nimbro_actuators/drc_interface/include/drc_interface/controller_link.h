// One serial connection to a µC controlling the servos
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef DRC_INTERFACE_CONTROLLER_LINK_H
#define DRC_INTERFACE_CONTROLLER_LINK_H

#include <robotcontrol/model/robotmodel.h>

#include <drc_interface/joint.h>

#include <ros/ros.h>
#include <plot_msgs/Plot.h>

namespace drc_interface
{

class ControllerLinkPrivate;

class ControllerLink
{
public:
	ControllerLink(robotcontrol::RobotModel* model, const std::string& prefix);
	~ControllerLink();

	void addJoint(const Joint::Ptr& joint);

	bool init(const std::string& filename);

	bool writeChar(uint8_t c);

	bool sendCommands(bool softwareStop);
	bool readFeedback();

	void setStiffness(float stiffness);
	
	void fadeTorque(float torque_proportion);
	bool fadeTorqueFinished();
	void fadeJoints(const std::vector<Joint::Ptr>& joints, float torqueProportion, uint32_t fade_time_ms);
	bool fadeJointsFinished();
	bool emergencyStopActive();

	bool rebootServo(const Joint::Ptr& joint);
	bool setPGain(const Joint::Ptr& joint, const uint16_t gain);
	bool reconfigureController();
	bool silenceHand(uint8_t silenceSequence);

	const std::string& path() const;
private:
	bool connect();
	bool configure();

	void sendBuf();
	bool readMsg(unsigned int timeout_usec = 100*1000, unsigned int* remaining = 0);

	ControllerLinkPrivate* m_d;
};

}

#endif
