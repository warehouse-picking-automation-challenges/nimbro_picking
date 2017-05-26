// One serial connection to a µC controlling the servos
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <drc_interface/controller_link.h>

#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <robotcontrol/model/robotmodel.h>
#include <Eigen/Geometry>

#include <std_msgs/UInt16.h>

#include <angles/angles.h>

#include "drc_proto.h"
#include "../protocol/drc_proto_constants.h"

#include <libucomm/envelope.h>
#include <libucomm/checksum.h>

#include <errno.h>
#include <fcntl.h>
#include <linux/serial.h>
#include <sstream>
#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/poll.h>
#include <termios.h>


#define COMM_DEBUG 0

namespace drc_interface
{

typedef uc::EnvelopeWriter<uc::InvertedModSumGenerator, ControllerLink> EnvelopeWriter;
typedef uc::IO<EnvelopeWriter, uc::IO_W> SimpleWriter;
typedef Proto<SimpleWriter> WProto;

typedef uc::EnvelopeReader<uc::InvertedModSumGenerator, 1024> EnvelopeReader;
typedef uc::IO<EnvelopeReader, uc::IO_R> SimpleReader;
typedef Proto<SimpleReader> RProto;

class ControllerLinkPrivate
{
public:
	ControllerLinkPrivate(robotcontrol::RobotModel* model, const std::string& prefix)
	 : model(model)
	 , rxBuf(1024)
	 , rxSize(0)
	 , commAllowed(true)
	 , param_ttl_base("/robotcontrol/" + prefix + "/ttl_base_id", 0, 1, 255, 7)
	{}

	robotcontrol::RobotModel* model;
	robotcontrol::RobotModel::State unsafeConnectionState;

	std::string path;
	int fd;

	std::vector<Joint::Ptr> joints;
	std::vector<Joint::Ptr> idToJoint;

	ros::Time last_imu_stamp;

	std::vector<uint8_t> txBuf;

	std::vector<uint8_t> rxBuf;
	unsigned int rxSize;
	uint16_t requestID;

	ros::Time feedbackRequestStamp;
	bool fadeTorqueFinished;
	bool fadeJointsFinished;
	bool emergencyStopActive;
	bool commAllowed;

	config_server::Parameter<int> param_ttl_base;

	ros::NodeHandle* nh;
	ros::Publisher distance_pub;

	EnvelopeReader reader;
};

ControllerLink::ControllerLink(robotcontrol::RobotModel* model, const std::string& prefix)
 : m_d(new ControllerLinkPrivate(model, prefix))
{
	m_d->unsafeConnectionState = model->registerState("unsafe_connection");
	m_d->nh = new ros::NodeHandle;
	m_d->distance_pub = m_d->nh->advertise<std_msgs::UInt16>("/momaro/" + prefix + "/distance", 1);

	m_d->requestID = 0;
}

ControllerLink::~ControllerLink()
{
	delete m_d->nh;
	delete m_d;
}

void ControllerLink::addJoint(const Joint::Ptr& joint)
{
	if(joint->id() == 0)
		return;

	m_d->joints.push_back(joint);

	if(joint->id() >= (int)m_d->idToJoint.size())
		m_d->idToJoint.resize(joint->id()+1);

	m_d->idToJoint[joint->id()] = joint;

	joint->link = this;
}

bool ControllerLink::connect()
{
	EnvelopeWriter output(this);

	for(int tries = 0; tries < 5; ++tries)
	{
		WProto::ConnectMsg msg;
		output << msg;
		sendBuf();

		if(!readMsg())
			continue;

		if(m_d->reader.msgCode() != RProto::ConnectMsgReply::MSG_CODE)
		{
			ROS_WARN_THROTTLE(1.0,
				"Unexpected msg with code %d while trying to connect...",
				m_d->reader.msgCode()
			);
			continue;
		}

		RProto::ConnectMsgReply reply;
		reply.version = 255;
		m_d->reader >> reply;

		if(reply.version != PROTOCOL_VERSION)
		{
			ROS_ERROR("%s: Firmware version differs from the one I'm built for: µC: %u, me: %u", m_d->path.c_str(), reply.version, PROTOCOL_VERSION);
			return false;
		}

		if(reply.error & CONNECT_MSG_ERROR_FLAG_NOT_SHUT_DOWN)
		{
			ROS_WARN("#------------------ CAUTION! HARDWARE AT RISK! ------------ ACTUATORS STILL UNDER POWER ----------------------#");
			ROS_WARN("%s: Will connect to unsafe state: (at least one) limb(s) was/were not shut down properly.", m_d->path.c_str());
			ROS_WARN("This happened on connect.");
			ROS_WARN("#-------------------------------------------------------------------------------------------------------------#");
			m_d->model->setState(m_d->unsafeConnectionState);
			return true;
		}
		else if(reply.error == 0)
		{
			ROS_INFO("%s: Connected to controller (firmware version %u)", m_d->path.c_str(), reply.version);
			return true;
		}
	}

	ROS_ERROR("%s: No answer to ConnectMsg packet.", m_d->path.c_str());
	return false;
}

bool ControllerLink::configure()
{
	//if(m_model->state() == m_unsafeConnectionState)
	//{
		//ROS_INFO("%s: Will not re-configure servos, since some of them already have torque.", m_d->path.c_str());
		//return true;
	//}

	EnvelopeWriter output(this);

	WProto::ServoConfigurationMsg cfg;
	cfg.ttl_bus_base_id = m_d->param_ttl_base();
	cfg.ttl_bus_address = 29;

	std::vector<WProto::ServoConfiguration> servos(m_d->joints.size());

	for(unsigned int i = 0; i < m_d->joints.size(); ++i)
	{
		const Joint::Ptr& joint = m_d->joints[i];
		servos[i].id = joint->id();
		servos[i].flags = 0;
		if(joint->velocityMode())
			servos[i].flags |= SERVO_FLAG_VELOCITY_CONTROL;
		if(joint->isTTL())
			servos[i].flags |= SERVO_FLAG_IS_TTL;
		servos[i].max_velocity = joint->maxVelocity();
		servos[i].fadein_velocity = joint->fadeInVelocity();

		m_d->joints[i]->goalTorque = 0;
		ROS_DEBUG("Servo '%20s': ID %2d, TTL: %d", joint->name.c_str(), joint->id(), joint->isTTL());
	}

	cfg.servos.setData(servos.data(), servos.size());

	output << cfg;
	sendBuf();

	if(!readMsg(20000 * 1000))
	{
		ROS_ERROR_THROTTLE(1.0, "%s: Could not read reply to servo configuration", m_d->path.c_str());
		return false;
	}

	if(m_d->reader.msgCode() != RProto::ServoConfiguredMsg::MSG_CODE)
	{
		ROS_ERROR_THROTTLE(1.0, "%s: Invalid reply to configuration request (%d)", m_d->path.c_str(), m_d->reader.msgCode());
		return false;
	}

	RProto::ServoConfiguredMsg reply = {};
	if(!m_d->reader.read(&reply))
	{
		ROS_ERROR_THROTTLE(1.0, "%s: Invalid reply to configure()", m_d->path.c_str());
		return false;
	}

	if(reply.error == CONFIGURE_MSG_ERROR_FLAG_NOT_SHUT_DOWN)
	{
		ROS_WARN("#------------------ CAUTION! HARDWARE AT RISK! ------------ ACTUATORS STILL UNDER POWER ----------------------#");
		ROS_WARN("%s: Will connect to unsafe state: (at least one) limb(s) was/were not shut down properly.", m_d->path.c_str());
		ROS_WARN("This happened on configure.");
		ROS_WARN("Servo mask: %X", reply.servo_mask);
		ROS_WARN("#-------------------------------------------------------------------------------------------------------------#");
		m_d->model->setState(m_d->unsafeConnectionState);
	}

	ROS_INFO("%s: Configured for %u servos", m_d->path.c_str(), (unsigned int)servos.size());
	m_d->commAllowed = true;

	return true;
}

bool ControllerLink::init(const std::string& filename)
{
	m_d->path = filename;

	struct termios config;

	m_d->fd = open(filename.c_str(), O_RDWR | O_NOCTTY);
	if(m_d->fd < 0)
	{
		ROS_ERROR("%s: Could not open serial connection: '%s'", filename.c_str(), strerror(errno));
		return false;
	}

	// Lock the serial connection
	if(lockf(m_d->fd, F_TLOCK, 0) != 0)
	{
		ROS_ERROR("%s: Could not acquire serial lock (is another robotcontrol running?): %s", filename.c_str(), strerror(errno));
		return false;
	}

	// Initialise the terminal interface struct
	memset(&config, 0, sizeof(config));
	if(tcgetattr(m_d->fd, &config) != 0)
	{
		ROS_ERROR("Could not get terminal attributes: %s", strerror(errno));
		close(m_d->fd);
		m_d->fd = -1;
		return false;
	}

	cfmakeraw(&config);
	cfsetspeed(&config, B115200);

	// Set the required terminal attributes
	if(tcsetattr(m_d->fd, TCSANOW, &config) != 0)
	{
		ROS_ERROR("Could not set terminal attributes: %s", strerror(errno));
		close(m_d->fd);
		m_d->fd = -1;
		return false;
	}

	// Try to connect
	if(!connect())
	{
		ROS_ERROR("%s: Could not connect to µC", m_d->path.c_str());
		close(m_d->fd);
		m_d->fd = -1;
		return false;
	}

	// Send servo configuration to µC
	if(!configure())
	{
		ROS_ERROR("%s: Could not send servo configuration", m_d->path.c_str());
		close(m_d->fd);
		m_d->fd = -1;
		return false;
	}

	return true;
}

bool ControllerLink::writeChar(uint8_t c)
{
	m_d->txBuf.push_back(c);
	return true;
}

void ControllerLink::sendBuf()
{
#if COMM_DEBUG
	printf("TX:");
	for(unsigned int i = 0; i < m_d->txBuf.size(); ++i)
		printf(" %02x", m_d->txBuf[i]);
	printf("\n");
#endif

	if(write(m_d->fd, m_d->txBuf.data(), m_d->txBuf.size()) != (int)m_d->txBuf.size())
	{
		ROS_ERROR("Could not write on serial port: %s", strerror(errno));
	}

	m_d->txBuf.clear();
}

bool ControllerLink::readMsg(unsigned int timeout_usec, unsigned int* remaining)
{
	timeval timeout;
	timeout.tv_sec = timeout_usec / 1000000;
	timeout.tv_usec = timeout_usec % 1000000;

	while(1)
	{
		unsigned int i;
		for(i = 0; i < m_d->rxSize; ++i)
		{
			if(m_d->reader.take(m_d->rxBuf[i]) == EnvelopeReader::NEW_MESSAGE)
			{
				m_d->rxSize = m_d->rxSize - 1 - i;
				memmove(&m_d->rxBuf[0], &m_d->rxBuf[i+1], m_d->rxSize);

				if(remaining)
					*remaining = timeout.tv_sec * 1000000 + timeout.tv_usec;

				return true;
			}
		}

		fd_set fds;
		FD_ZERO(&fds);
		FD_SET(m_d->fd, &fds);

		int ret = select(m_d->fd+1, &fds, 0, 0, &timeout);
		if(ret < 0)
		{
			if(errno == EAGAIN || errno == EINTR)
				continue;

			ROS_ERROR_THROTTLE(1.0, "select() fail: %s", strerror(errno));
			return false;
		}

		if(ret == 0 || !FD_ISSET(m_d->fd, &fds))
		{
			ROS_INFO("%s: timeout", m_d->path.c_str());
			return false; // timeout
		}

		ret = read(m_d->fd, m_d->rxBuf.data(), m_d->rxBuf.size());
		if(ret < 0)
		{
			ROS_ERROR_THROTTLE(1.0, "read() fail: %s", strerror(errno));
			return false;
		}
		if(ret == 0)
		{
			ROS_ERROR("Got size 0 from read");
			std::abort();
		}

		m_d->rxSize = ret;

#if COMM_DEBUG
		printf("RX:");
		for(unsigned int i = 0; i < m_d->rxSize; ++i)
			printf(" %02x", m_d->rxBuf[i]);
		printf("\n");
#endif
	}

	return false;
}

bool ControllerLink::sendCommands(bool softwareStop)
{
	EnvelopeWriter output(this);

	WProto::ServoCommandMsg msg;

	// Push the joints that a likely to have timeouts further back in the list
	// so they won't block communication to the working servos as much
	for (std::size_t i = 0; i < m_d->joints.size() - 1; ++i)
	{
		if (m_d->joints[i]->timeoutConfidence > m_d->joints[i+1]->timeoutConfidence)
		{
#if COMM_DEBUG
			ROS_INFO("Joint %d is being pushed back in the list from index %d to %d",
				m_d->joints[i]->id(),
				(unsigned int)i,
				(unsigned int)i+1
			);
#endif
			Joint::Ptr temp = m_d->joints[i+1];
			m_d->joints[i+1] = m_d->joints[i];
			m_d->joints[i] = temp;

		}
	}

	std::vector<WProto::ServoCommand> cmds(m_d->joints.size());
	for(unsigned int i = 0; i < m_d->joints.size(); ++i)
	{
		const Joint& joint = *m_d->joints[i];
		cmds[i].id = joint.id();

		if(joint.velocityMode())
		{
			float cmd = joint.cmd.vel;
			if(joint.invert())
				cmd = -cmd;

			int32_t ticks = ((cmd / (2.0 * M_PI)) * 60) * joint.gearboxRatio();

			if(ticks >= joint.maxVelocity() || ticks <= -joint.maxVelocity())
			{
				int32_t old = ticks;
				if(ticks >= joint.maxVelocity())
					ticks = joint.maxVelocity();
				else if(ticks <= -joint.maxVelocity())
					ticks = -joint.maxVelocity();

				ROS_ERROR("Velocity controlled joint '%s' out of bounds! Constraining tick value %d (%f rad/s) to %d!",
					joint.name.c_str(),
					old, joint.cmd.vel,
					ticks
				);
			}

			cmds[i].command = ticks;
		}
		else
		{
			float cmd = joint.cmd.pos;

			// Enforce urdf position limits
			if(joint.modelJoint->type == urdf::Joint::REVOLUTE)
			{
				const boost::shared_ptr<urdf::JointLimits>& limits = joint.modelJoint->limits;
				if(limits)
				{
					if(cmd > limits->upper)
					{
						ROS_ERROR_THROTTLE(1.0, "Position controlled joint '%s' out of bounds! Constraining position %f to upper limit %f!",
							joint.name.c_str(),
							cmd, limits->upper
						);
						cmd = limits->upper;
					}
					else if(cmd < limits->lower)
					{
						ROS_ERROR_THROTTLE(1.0, "Position controlled joint '%s' out of bounds! Constraining position %f to lower limit %f!",
							joint.name.c_str(),
							cmd, limits->lower
						);
						cmd = limits->lower;
					}
				}
			}

			if(joint.invert())
				cmd = -cmd;
			cmds[i].command = cmd / (2.0 * M_PI) * joint.encoderTicks() + joint.offset();
		}
	}

	msg.servos.setData(cmds.data(), cmds.size());

	msg.software_stop = softwareStop;

	m_d->requestID++;
	msg.request_id = m_d->requestID;

	m_d->rxSize = 0;

	if(m_d->commAllowed)
	{
		output << msg;
		sendBuf();
	}

	m_d->feedbackRequestStamp = ros::Time::now();

	return true;
}

bool ControllerLink::readFeedback()
{
	RProto::ServoFeedbackMsg msg;

	unsigned int budget_usec = 100 * 1000;
	while(1)
	{
		if(!readMsg(budget_usec, &budget_usec))
		{
			ROS_ERROR_THROTTLE(1.0, "%s: timeout", m_d->path.c_str());
			return false;
		}

		if(m_d->reader.msgCode() != RProto::ServoFeedbackMsg::MSG_CODE)
		{
			ROS_ERROR_THROTTLE(1.0, "%s: unexpected packet %d", m_d->path.c_str(), m_d->reader.msgCode());
			return false;
		}

		m_d->reader >> msg;
		if(msg.request_id == m_d->requestID)
			break;
	}

	m_d->emergencyStopActive = (msg.error & SERVO_FEEDBACK_ERROR_EMERGENCY_PRESSED);

	if(msg.error != SERVO_FEEDBACK_ERROR_NONE)
	{
		ROS_ERROR_THROTTLE(1.0, "%s: feedback error %d", m_d->path.c_str(), msg.error);
		if(msg.error != SERVO_FEEDBACK_ERROR_EMERGENCY_PRESSED)
			return false;
	}

	RProto::ServoFeedback fb;
	memset(&fb, 0, sizeof(fb));

	bool timeoutHappened = false;
	unsigned int newTimeouts = 0;

	while(msg.servos.next(&fb))
	{
		if(fb.id >= m_d->idToJoint.size())
		{
			ROS_WARN_THROTTLE(1.0, "%s: Invalid joint id %d", m_d->path.c_str(), fb.id);
			continue;
		}

		Joint::Ptr joint = m_d->idToJoint[fb.id];

		if(!joint)
		{
			//ROS_WARN_THROTTLE(1.0, "%s: feedback on unknown joint ID %d", m_d->path.c_str(), fb.id);
			ROS_ERROR_THROTTLE(5.0, "---------------------------------------------------------------------------");
			ROS_ERROR_THROTTLE(5.0, "---------------------------------------------------------------------------");
			ROS_ERROR_THROTTLE(5.0, "---------------------------!!CAUTION!!-------------------------------------");
			ROS_ERROR_THROTTLE(5.0, "%s: feedback on unknown joint ID %d, possible microcontroller error!", m_d->path.c_str(), fb.id);
			ROS_ERROR_THROTTLE(5.0, "       Will not allow (fading)commands to be send to microcontroller!      ");
			ROS_ERROR_THROTTLE(5.0, "---------------------------------------------------------------------------");
			ROS_ERROR_THROTTLE(5.0, "---------------------------------------------------------------------------");
			ROS_ERROR_THROTTLE(5.0, "---------------------------------------------------------------------------");
			m_d->commAllowed = false;
			continue;
		}


		if (joint->timeouts < fb.timeouts && !timeoutHappened)
		{
			timeoutHappened = true;
			newTimeouts = 1;
#if COMM_DEBUG
			ROS_INFO("Servo %d was the first in the chain to have a timeout",
				joint->id()
			);
#endif
		}
		else
			newTimeouts = 0; // Ignore all timeouts after the first
		joint->timeoutConfidence = joint->timeoutConfidence * 0.9 + newTimeouts * 0.1;


		joint->feedback.stamp = m_d->feedbackRequestStamp;
		joint->feedback.pos = 2.0 * M_PI * ((double)(fb.position - joint->offset())) / joint->encoderTicks(); // FIXME: ticks -> radians
		joint->feedback.torque = fb.torque / 1000.0 * joint->NmPerAmp();
		joint->temperature = fb.temperature;
		joint->timeouts = fb.timeouts;
		joint->error = fb.error;
		joint->voltage = fb.voltage + SERVO_VOLTAGE_OFFSET;


#if COMM_DEBUG
		ROS_INFO("Temp of servo %d is: %f / %d", joint->id(), joint->temperature, fb.temperature);
#endif

		if(joint->invert())
		{
			joint->feedback.pos = -joint->feedback.pos;
			joint->feedback.torque = -joint->feedback.torque;
		}
	}

	// save distance sensor value
	std_msgs::UInt16 distanceMsg;
	distanceMsg.data = msg.distance;
	m_d->distance_pub.publish(distanceMsg);

	return true;
}

void ControllerLink::setStiffness(float stiffness)
{
// 	EnvelopeWriter output(this);
// 
// 	WProto::FadeCommandMsg msg;
// 
// 	std::vector<WProto::FadeCommand> cmds(m_d->joints.size());
// 	for(unsigned int i = 0; i < m_d->joints.size(); ++i)
// 	{
// 		const Joint& joint = *m_d->joints[i];
// 
// 		cmds[i].id = joint.id();
// 
// 		uint16_t value = stiffness * 200;
// 		if(value == 0)
// 			value = 1;
// 
// 		cmds[i].torque = value;
// 	}
// 
// 	msg.servos.setData(cmds.data(), cmds.size());
// 
// 	output << msg;
// 	sendBuf();
}

void ControllerLink::fadeTorque(float torque_proportion)
{
	m_d->fadeTorqueFinished = false;
	EnvelopeWriter output(this);
	WProto::FadeCommandMsg msg;
	msg.fade_time = 10 * 1000;
	std::vector<WProto::FadeCommand> cmds(m_d->joints.size());
	for(unsigned int i = 0; i < m_d->joints.size(); ++i){
		Joint& joint = *m_d->joints[i];
		cmds[i].id = joint.id();
		cmds[i].torque = joint.maxTorque() * torque_proportion;
		//cmds[i].fadein_velocity = joint.fadeInVelocity();
		//cmds[i].final_velocity = joint.maxVelocity();
		joint.goalTorque = cmds[i].torque;
	}
	msg.servos.setData(cmds.data(), cmds.size());

	if(m_d->commAllowed)
	{
		output << msg;
		sendBuf();
	}
}

void ControllerLink::fadeJoints(const std::vector<Joint::Ptr>& joints, float torqueProportion, uint32_t fade_time_ms)
{
	m_d->fadeJointsFinished = false;
	EnvelopeWriter output(this);
	WProto::FadeCommandMsg msg;
	std::vector<WProto::FadeCommand> cmds;
	for(auto& joint : joints)
	{
		if(std::find(m_d->joints.begin(), m_d->joints.end(), joint) == m_d->joints.end())
			continue;

		WProto::FadeCommand cmd;
		cmd.id = joint->id();
		cmd.torque = torqueProportion * joint->maxTorque();
		//cmd.fadein_velocity = joint->fadeInVelocity();
		//cmd.final_velocity = joint->maxVelocity();
		joint->goalTorque = cmd.torque;
		cmds.push_back(cmd);
	}

	if(cmds.size() == 0)
		return; // Nothing to be done here.

	for(auto& cmd : cmds)
		ROS_INFO("ControllerLink '%s': Fading joint with ID %u to torque %u", m_d->path.c_str(), cmd.id, cmd.torque);

	msg.fade_time = fade_time_ms;
	msg.servos.setData(cmds.data(), cmds.size());
	if(m_d->commAllowed)
	{
		output << msg;
		sendBuf();
	}
}

bool ControllerLink::fadeJointsFinished()
{
	if(m_d->fadeJointsFinished)
		return true;

	if(!readMsg(100*1000))
		return false;

	if(m_d->reader.msgCode() != RProto::FadeCommandMsgReply::MSG_CODE)
	{
		ROS_ERROR_THROTTLE(1.0, "%s: Invalid reply to torque fade request (%d)", m_d->path.c_str(), m_d->reader.msgCode());
		return false;
	}

	ROS_INFO("%s: Torque fade finished", m_d->path.c_str());
	m_d->fadeTorqueFinished = true;
	return true;
}

bool ControllerLink::fadeTorqueFinished()
{
	if(m_d->fadeTorqueFinished)
		return true;
	else{
		if(!readMsg(100*1000))
		{
			return false;
		}

		if(m_d->reader.msgCode() != RProto::FadeCommandMsgReply::MSG_CODE)
		{
			ROS_ERROR_THROTTLE(1.0, "%s: Invalid reply to torque fade-in request (%d)", m_d->path.c_str(), m_d->reader.msgCode());
			return false;
		}

		ROS_INFO("%s: Torque fade-in finished", m_d->path.c_str());
		m_d->fadeTorqueFinished = true;
		return true;
	}
}

bool ControllerLink::emergencyStopActive()
{
	return m_d->emergencyStopActive;
}

bool ControllerLink::rebootServo(const Joint::Ptr& joint)
{
	if(joint->isTTL() || joint->id() <= 0)
		return false;

	EnvelopeWriter output(this);

	WProto::RebootServoMsg msg;
	msg.id = joint->id();

	output << msg;
	sendBuf();

	return true;
}


bool ControllerLink::setPGain(const Joint::Ptr& joint, const uint16_t gain)
{
	if(joint->isTTL() || joint->id() <= 0)
		return false;

	EnvelopeWriter output(this);

	WProto::SetServoParamMsg msg;
	msg.id = joint->id();
	msg.pgain = gain;

	output << msg;
	sendBuf();

	return true;
}


bool ControllerLink::reconfigureController()
{
	ROS_INFO("::::: Reconfiguring Controller %s", m_d->path.c_str());
	ROS_WARN("::::: Rebooting microcontroller now! (This may take a second)");

	EnvelopeWriter output(this);

	WProto::RebootControllerMsg msg;
	msg.key1 = REBOOT_KEY1;
	msg.key2 = REBOOT_KEY2;
	output << msg;
	sendBuf();
	
	// This will also stall robotcontrol! 
	usleep(1500 * 1000);

	ROS_INFO("::::: Connecting");
	if (!connect())
	{
		ROS_ERROR("::::: Could not connect to Microcontroller!");
		return false;
	}
	ROS_INFO("::::: Configure");
	if(!configure())
	{
		ROS_ERROR("::::: Could not configure Microcontroller!");
		return false;
	}
	ROS_INFO("::::: Reconfigure Done!");
	
	return true;
}

bool ControllerLink::silenceHand(uint8_t silenceSequence)
{
	EnvelopeWriter out(this);
	WProto::SetSilenceMsg msg;
	msg.silence_sequence = silenceSequence;

	out<< msg;
	sendBuf();

	return true;
}

const std::string& ControllerLink::path() const
{
	return m_d->path;
}


}
