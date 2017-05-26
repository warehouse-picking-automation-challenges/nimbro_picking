// Communication controller for APC
// Author: Sebastian Schüller<schuell1@cs.uni-bonn.de>


#include <apc_interface/apc_controller.h>

#include <apc_proto.h>
#include <libucomm/envelope.h>
#include <libucomm/checksum.h>

#include <stdint.h>
#include <vector>

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <sys/select.h>

#include <ros/console.h>
#include <ros/assert.h>


typedef uc::EnvelopeWriter<uc::InvertedModSumGenerator, UARTBuffer> EnvelopeWriter;
typedef uc::IO<EnvelopeWriter, uc::IO_W> SimpleWriter;
typedef Proto<SimpleWriter> WProto;

typedef uc::EnvelopeReader<uc::InvertedModSumGenerator, 1024> EnvelopeReader;
typedef uc::IO<EnvelopeReader, uc::IO_R> SimpleReader;
typedef Proto<SimpleReader> RProto;

constexpr bool DEBUG_COMM = false;


// NOTE(sebastian): This is a little bit stupid, but libucomm wants a class
// that implemets the writeChar method. This literally just wraps the
// vector push_back method.
struct UARTBuffer
{
	UARTBuffer() : tx(1024), rx(1024), rx_size(0){};
	bool writeChar(uint8_t c)
	{
		if(DEBUG_COMM)
			printf("Write 0x%02X\n", c);
		tx.push_back(c);
		return true;
	}
	EnvelopeReader msg;
	std::vector<uint8_t> tx;
	std::vector<uint8_t> rx;
	std::size_t rx_size;
};

namespace apc_interface
{

Controller::Controller()
 : m_fd(-1)
 , m_buffer(new UARTBuffer())
{
}

Controller::~Controller()
{
	close(m_fd);
	m_fd = -1;
}

bool Controller::initUART(std::string filename)
{
	m_fd = open(filename.c_str(), O_RDWR | O_NOCTTY);
	if(m_fd < 0)
	{
		ROS_ERROR("Could not open serial connection: '%s'", strerror(errno));
		return false;
	}
	if(lockf(m_fd, F_TLOCK, 0) != 0)
	{
		ROS_ERROR("Could not lock serial connection: '%s'", strerror(errno));
		goto connect_failed;
	}

	struct termios config;
	memset(&config, 0, sizeof(config));
	if(tcgetattr(m_fd, &config) != 0)
	{
		ROS_ERROR("Could not get terminal attributes: '%s'", strerror(errno));
		goto connect_failed;
	}
	cfmakeraw(&config);
	cfsetspeed(&config, B115200);
	if(tcsetattr(m_fd, TCSANOW,&config) != 0)
	{
		ROS_ERROR("Could not set terminal attributes: '%s'", strerror(errno));
		goto connect_failed;
	}


	return true;

connect_failed:
	close(m_fd);
	m_fd = -1;
	return false;
}

bool Controller::connect()
{
	EnvelopeWriter output(m_buffer.get());
	WProto::ConnectRequestMsg msg;
	RProto::ConnectAck ack;
	memset(&ack, 0, sizeof(ack));


	output << msg;
	if(!sendBuffer())
		return false;
	if(!readMsg())
	{
		ROS_ERROR("Got no connect response from controller.");
		return false;
	}
	if(m_buffer->msg.msgCode() != RProto::ConnectAck::MSG_CODE)
	{
		ROS_ERROR("Unexpected responce to connect request.");
		return false;
	}
	m_buffer->msg >> ack;
	if(ack.protocol_version != ProtoConstants::PROTOCOL_VERSION)
	{
		ROS_ERROR("Controller uses an older version of the protocol than the software.");
		ROS_ERROR("Please update the controller-firmware.");
		ROS_ERROR("Used protocol versions: Software: %d, Hardware: %d",
			ProtoConstants::PROTOCOL_VERSION,
			ack.protocol_version
		);
		return false;
	}
	return true;
}

bool Controller::initializeServos(std::vector<uint16_t> ids)
{
	if(ids.size() > ProtoConstants::NUM_SERVOS)
	{
		ROS_ERROR( "Trying to initialize more servos than the protocol thinks we have.");
		return false;
	}
	EnvelopeWriter output(m_buffer.get());
	WProto::ServoInitializationMsg msg = {};
	for(size_t i = 0; i < ids.size(); ++i)
		msg.ids[i] = ids[i];
	output << msg;
	if(!sendBuffer())
		return false;

	return true;
}

bool Controller::rebootController()
{
	EnvelopeWriter output(m_buffer.get());
	WProto::RebootControllerMsg msg;
	msg.key1 = ProtoConstants::REBOOT_KEY1;
	msg.key2 = ProtoConstants::REBOOT_KEY2;
	output << msg;

	if(!sendBuffer())
		return false;

	if(!readMsg())
	{
		ROS_ERROR("Reboot request didn't recieve an ACK from controller.");
		return false;
	}

	if(m_buffer->msg.msgCode() != RProto::RebootControllerAck::MSG_CODE)
	{
		ROS_ERROR("Unexpected response for reboot request");
		return false;
	}

	RProto::RebootControllerAck ack;
	memset(&ack, 0, sizeof(ack));
	m_buffer->msg >> ack;
	if(ack.status == RebootControllerMsgStatus::WRONG_KEY)
	{
		ROS_ERROR("Controller got wrong reboot key. Did you want to reboot?");
		return false;
	}

	ROS_INFO("Rebooted controller");
	return true;
}

bool Controller::dimLEDs(uint8_t duty)
{
	EnvelopeWriter out(m_buffer.get());
	WProto::DimLightMsg msg = {};
	msg.duty = duty;
	out << msg;

	return sendBuffer();
}

bool Controller::switchVacuum(bool on)
{
	EnvelopeWriter out(m_buffer.get());
	WProto::SwitchVacuumMsg msg = {};
	if(on)
		msg.on = 1;
	else
		msg.on = 0;
	out << msg;
	if(!sendBuffer())
		return false;
	return true;
}

bool Controller::switchVacuumPower(bool on)
{
	EnvelopeWriter out(m_buffer.get());
	WProto::SwitchVacuumPowerMsg msg = {};
	if(on)
		msg.on = 1;
	else
		msg.on = 0;
	out << msg;
	if(!sendBuffer())
		return false;
	return true;
}

bool Controller::sendPositionCmds(const ControllerStatus& status)
{
	if(status.servos.size() > ProtoConstants::NUM_SERVOS)
	{
		throw std::logic_error("sendPositionCmds: got too many servos");
	}

	EnvelopeWriter out(m_buffer.get());
	WProto::ServoPositionCmdMsg msg = {};
	for(size_t i = 0; i < status.servos.size(); ++i)
	{
		const auto& servo = status.servos[i];
		msg.cmds[i].id = servo.id;
		msg.cmds[i].goal_position = servo.goal_position_ticks;
	}
	out << msg;
	if(!sendBuffer())
	{
		ROS_ERROR("Could not send data");
		return false;
	}

	return true;
}

bool Controller::sendTorqueEnable(const ControllerStatus& status)
{
	if(status.servos.size() > ProtoConstants::NUM_SERVOS)
		return false;
	EnvelopeWriter out(m_buffer.get());
	WProto::ServoTorqueEnableMsg msg = {};
	for(size_t i = 0; i < status.servos.size(); ++i)
	{
		const auto& servo = status.servos[i];
		msg.cmds[i].id = servo.id;
		msg.cmds[i].enable = servo.torque_enabled;
	}
	out << msg;
	if(!sendBuffer())
		return false;
	return true;
}

bool Controller::sendMaxTorque(const ControllerStatus& status)
{
	if(status.servos.size() > ProtoConstants::NUM_SERVOS)
	{
		ROS_ERROR("Failed to send maxTorque, too many commands");
		return false;
	}

	EnvelopeWriter out(m_buffer.get());
	WProto::ServoMaxTorqueMsg msg = {};

	for(size_t i = 0; i < status.servos.size(); ++i)
	{
		const auto& servo = status.servos[i];

		msg.cmds[i].id = servo.id;
		msg.cmds[i].torque = servo.max_torque;

		ROS_INFO("Setting torque of servo '%s': %d", servo.name.c_str(), servo.max_torque);
	}

	out << msg;
	if(!sendBuffer())
		return false;

	return true;
}


bool Controller::updateStatus(ControllerStatus* status)
{
	if(!status)
		return false;

	EnvelopeWriter out(m_buffer.get());
	WProto::ControllerStatusRqst rqst = {};
	out << rqst;

	if(!sendBuffer())
		return false;
	if(!readMsg())
		return false;
	if(m_buffer->msg.msgCode() != RProto::ControllerStatusMsg::MSG_CODE)
		return false;

	RProto::ControllerStatusMsg stat= {};
	m_buffer->msg >> stat;
	status->vacuum_on = (stat.status & ControllerStatusFlags::VACUUM_ON);
	status->vacuum_power_on = (stat.status & ControllerStatusFlags::VACUUM_POWER_ON);
	status->light_duty = stat.light_duty;
	status->air_pressure = stat.air_pressure;
	status->status = stat.status;

	for(size_t i = 0; i < ProtoConstants::NUM_SERVOS; ++i)
	{
		if(i >= status->servos.size())
			break;
		if(status->servos[i].id != stat.servos[i].id)
			throw std::logic_error("Servo ID from libucomm packet doesn't fit to local servo-array ID");
		status->servos[i].flags = stat.servos[i].flags;
		status->servos[i].goal_position_ticks = stat.servos[i].goal_position;
		status->servos[i].position_ticks = stat.servos[i].position;
		status->servos[i].velocity_ticks = stat.servos[i].speed;
		status->servos[i].load = stat.servos[i].load;
		status->servos[i].voltage = stat.servos[i].voltage;
		status->servos[i].temperature = stat.servos[i].temperature;
		status->servos[i].error = stat.servos[i].error;
	}
	return true;
}

bool Controller::updateEEPROM(ControllerStatus* status)
{
	if(!status)
		return false;
	EnvelopeWriter out(m_buffer.get());
	WProto::EEPROMStatusRqst rqst = {};
	out << rqst;

	if(!sendBuffer())
		return false;
	if(!readMsg())
		return false;
	if(m_buffer->msg.msgCode() != RProto::EEPROMStatusMsg::MSG_CODE)
		return false;

	RProto::EEPROMStatusMsg stat = {};
	m_buffer->msg >> stat;

	for(size_t i = 0; i < ProtoConstants::NUM_SERVOS; ++i)
	{
		if(i >= status->servos.size())
			break;
		if(status->servos[i].id != stat.eeprom[i].id)
			throw std::logic_error("Servo ID from libucomm packet doesn't fit to local servo-array ID");
		status->servos[i].max_torque = stat.eeprom[i].max_torque;
	}
	return true;
}

bool Controller::getDbgStatus(ControllerDebugStatus* status)
{
	if(!status)
		return false;

	EnvelopeWriter out(m_buffer.get());
	WProto::DbgStatusRqst rqst = {};
	out << rqst;

	if(!sendBuffer())
		return false;
	if(!readMsg())
		return false;
	if(m_buffer->msg.msgCode() != RProto::DbgStatusMsg::MSG_CODE)
		return false;

	RProto::DbgStatusMsg msg = {};
	m_buffer->msg >> msg;

	ROS_WARN("%x" , msg.dxl_errors);
	ROS_WARN("%x" , msg.comm_errors);
	status->comm_statuscheck_uninitialized =
		(msg.comm_errors & DbgStatusFlags::COMM_STATUS_UNINITIALIZED);
	status->comm_poscmd_uninitialized =
		(msg.comm_errors & DbgStatusFlags::COMM_POSCMD_UNINITIALIZED);
	status->comm_poscmd_invld_id =
		(msg.comm_errors & DbgStatusFlags::COMM_POSCMD_INVLD_ID);
	status->dxl_bulkread_invld_id =
		(msg.dxl_errors & DbgStatusFlags::DXL_BR_INVLD_ID);
	status->dxl_bulkread_invld_size =
		(msg.dxl_errors & DbgStatusFlags::DXL_BR_INVLD_SIZE);

	return true;
}



bool Controller::readMsg(unsigned int timeout_usec)
{
	timeval timeout;
	timeout.tv_sec = timeout_usec / 1000000;
	timeout.tv_usec = timeout_usec % 1000000;
	while(1)
	{
		unsigned int i;
		for(i = 0; i < m_buffer->rx_size; ++i)
		{
			if(m_buffer->msg.take(m_buffer->rx[i]) == EnvelopeReader::NEW_MESSAGE)
			{
				m_buffer->rx_size -= (i + 1);
				memmove(&m_buffer->rx[0], &m_buffer->rx[i+1], m_buffer->rx_size);
				return true;
			}
		}


		fd_set fds;
		FD_ZERO(&fds);
		FD_SET(m_fd, &fds);
		int ret = select(m_fd + 1, &fds, 0, 0, &timeout);
		if(ret < 0)
		{
			if(errno == EAGAIN || errno == EINTR)
				continue;

			ROS_ERROR_THROTTLE(1.0, "select() fail: %s", strerror(errno));
			return false;
		}
		if(ret == 0 || !FD_ISSET(m_fd, &fds))
		{
			ROS_INFO("Timeout on serial connection.");
			return false; // timeout
		}

		ret = read(m_fd, m_buffer->rx.data(), m_buffer->rx.size());
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

		if(DEBUG_COMM)
		{
			printf("RX:");
			for(int i = 0; i < ret; ++i)
				printf(" 0x%02X", m_buffer->rx.data()[i]);
			printf("\n");
		}

		m_buffer->rx_size = ret;
	}
}

bool Controller::sendBuffer()
{
	bool ret = true;
	auto size = m_buffer->tx.size();
	if(write(m_fd, m_buffer->tx.data(), size) != (int)size)
	{
		ROS_ERROR( "Could not write on serial connection: '%s'", strerror(errno));
		ret = false;
	}
	m_buffer->tx.clear();
	return ret;
}




}
