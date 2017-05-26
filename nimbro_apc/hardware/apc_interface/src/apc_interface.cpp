// ROS Interface to the APC hardware controller
// Author: Sebastian Schüller<schuell1@cs.uni-bonn.de>

#include <mutex>

#include <ros/init.h>
#include <ros/assert.h>
#include <ros/rate.h>

#include <ros/node_handle.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>

#include <apc_interface/ControllerState.h>
#include <apc_interface/DimLight.h>
#include <apc_interface/SuctionStrength.h>
#include <apc_interface/apc_controller.h>

#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <controller_manager/controller_manager.h>

#include <plot_msgs/Plot.h>
#include <plot_msgs/JointCommand.h>

#include <config_server/parameter.h>

#include <apc_proto.h>
#include <sched.h>

#if defined(__i386__)
#define __NR_ioprio_set         289
#define __NR_ioprio_get         290
#elif defined(__ppc__)
#define __NR_ioprio_set         273
#define __NR_ioprio_get         274
#elif defined(__x86_64__)
#define __NR_ioprio_set         251
#define __NR_ioprio_get         252
#elif defined(__ia64__)
#define __NR_ioprio_set         1274
#define __NR_ioprio_get         1275
#else
#error "Unsupported arch"
#endif

#define IOPRIO_CLASS_SHIFT      13


namespace
{
constexpr double diameter = .04;
constexpr double meterPerTick = ((M_PI * diameter) / 4096);
constexpr double radPerTick = ((M_PI * 2) / 4096);
int16_t meterToTicks(double value)
{
	return static_cast<int16_t>(value / meterPerTick);
}

int16_t radToTicks(double value)
{
	return static_cast<int16_t>(value / radPerTick);
}

double ticksToMeter(int16_t value)
{
	return static_cast<double>(value * meterPerTick);
}

double ticksToRad(int16_t value)
{
	return static_cast<double>(value * radPerTick);
}

constexpr char releaseServoName[] = "release_vacuum";

static inline int ioprio_set(int which, int who, int ioprio)
{
        return syscall(__NR_ioprio_set, which, who, ioprio);
}

enum {
        IOPRIO_CLASS_NONE,
        IOPRIO_CLASS_RT,
        IOPRIO_CLASS_BE,
        IOPRIO_CLASS_IDLE,
};

enum {
        IOPRIO_WHO_PROCESS = 1,
        IOPRIO_WHO_PGRP,
        IOPRIO_WHO_USER,
};

}

namespace apc_interface
{

class APCInterface : public hardware_interface::RobotHW
{
public:
	APCInterface();
	void setupServos(std::vector<ServoStatus> servos);
	void initMultiTurnOffset(const std::string& name);
	void writePos();
	void readFeedback();
	void shutdownServos();

private:
	std::mutex m_ctrl_lock;
	apc_interface::Controller m_ctrl;
	ControllerStatus m_status;


	ros::NodeHandle m_nh;

	ros::ServiceServer m_srv_rebootController;
	bool srv_rebootController(
		std_srvs::EmptyRequest& req,
		std_srvs::EmptyResponse& res
	);

	ros::ServiceServer m_srv_switchVacuum;
	bool srv_switchVacuum(
		std_srvs::SetBoolRequest& req,
		std_srvs::SetBoolResponse& res
	);

	ros::ServiceServer m_srv_switchVacuumPower;
	bool srv_switchVacuumPower(
		std_srvs::SetBoolRequest& req,
		std_srvs::SetBoolResponse& res
	);

	ros::ServiceServer m_srv_dimLight;
	bool srv_dimLight(
		apc_interface::DimLightRequest& req,
		apc_interface::DimLightResponse& res
	);

	ros::ServiceServer m_srv_suctionStrength;
	bool srv_setSuctionStrength(
		apc_interface::SuctionStrengthRequest& req,
		apc_interface::SuctionStrengthResponse& res
	);

	ros::ServiceServer m_srv_enableComm;
	bool srv_enableComm(
		std_srvs::SetBoolRequest& req,
		std_srvs::SetBoolResponse& res
	);

	ros::Publisher m_pub_status;
	ros::Publisher m_pub_plot;
	ros::Publisher m_pub_cmdPlot;

	hardware_interface::JointStateInterface m_jointStateInterface;
	hardware_interface::PositionJointInterface m_jointPosInterface;

	config_server::Parameter<float> m_param_airVel_alpha;
	config_server::Parameter<float> m_param_airVel_alpha_lower;
	config_server::Parameter<float> m_param_airVel_onTipThreshold;
	config_server::Parameter<float> m_param_airVel_wellAttachedThreshold;
	config_server::Parameter<float> m_param_airVel_minTime;
	config_server::Parameter<int>   m_param_multiTurnInitThreshold;
	config_server::Parameter<float> m_param_multiTurnInitTime;
	double m_airVel = 0.0;
	double m_airVelLowPass = 0.0;
	ros::Time m_lastOffTime;
	bool m_somethingOnTip = false;
	bool m_objectWellAttached  = false;
	double m_releaseOpening = 0.0;
	int m_releaseServoIndex = -1;
	bool m_commEnabled = true;
#if 0
	FILE* m_logFile;
#endif
};

void APCInterface::setupServos(std::vector<ServoStatus> servos)
{
	m_status.servos = servos;
	for(size_t i = 0; i < servos.size(); ++i)
	{
		if(servos[i].name == releaseServoName)
		{
			m_releaseServoIndex = i;
			break;
		}
	}

	if(m_releaseServoIndex < 0)
		throw std::logic_error("Release Servonot configured!");


	for(auto& servo : m_status.servos)
	{
		if(servo.name == releaseServoName)
			continue;
		m_jointStateInterface.registerHandle(
			hardware_interface::JointStateHandle(servo.name,
				&servo.position, &servo.velocity, &servo.effort)
		);
	}
	registerInterface(&m_jointStateInterface);

	for(auto& servo : m_status.servos)
	{
		if(servo.name == releaseServoName)
			continue;
		m_jointPosInterface.registerHandle(
			hardware_interface::JointHandle(
				m_jointStateInterface.getHandle(servo.name),
				&servo.goal_pos)
		);
	}
	registerInterface(&m_jointPosInterface);
	std::vector<uint16_t> ids;
	for(const auto& servo : m_status.servos)
		ids.push_back(servo.id);
	m_ctrl.initializeServos(ids);
	m_ctrl.updateStatus(&m_status);
	//m_ctrl.updateEEPROM(&m_status);
	// NOTE (sebastian): Use fixed torques for competition

	m_ctrl.sendMaxTorque(m_status);


	for(auto& s : m_status.servos)
		s.torque_enabled = true;

	m_ctrl.sendTorqueEnable(m_status);

}

void APCInterface::writePos()
{
	if(!m_commEnabled)
		return;

	{
		std::lock_guard<std::mutex> lock(m_ctrl_lock);
		m_status.servos[m_releaseServoIndex].goal_pos = m_releaseOpening;
	}

	plot_msgs::JointCommand cmdMsg;
	cmdMsg.header.stamp = ros::Time::now();

	for(auto& servo : m_status.servos)
	{
		double rawGoal = servo.goal_pos / servo.scaleToMetric;

		int16_t goal = 0;
		if(servo.type == ServoStatus::Type::LINEAR)
			goal = meterToTicks(rawGoal);
		else if(servo.type == ServoStatus::Type::REVOLUTE)
			goal = radToTicks(rawGoal);
		else
			ROS_BREAK();

		servo.goal_position_ticks = goal + servo.goal_position_offset;

		cmdMsg.name.push_back(servo.name);
		cmdMsg.position.push_back(rawGoal);
	}

	m_pub_cmdPlot.publish(cmdMsg);

	std::lock_guard<std::mutex> lock(m_ctrl_lock);

	if(!m_ctrl.sendPositionCmds(m_status))
		ROS_ERROR_THROTTLE(1,"Failed to write position commands!");
}

void APCInterface::readFeedback()
{
	if(!m_commEnabled)
		return;

	m_ctrl_lock.lock();
	if(!m_ctrl.updateStatus(&m_status))
		ROS_ERROR_THROTTLE(1, "Failed to read Feedback");
	m_ctrl_lock.unlock();
	for(auto& servo : m_status.servos)
	{
		double pos = 0.0;

		int32_t ticks = servo.position_ticks - servo.goal_position_offset;

		if(servo.type == ServoStatus::Type::LINEAR)
			pos = ticksToMeter(ticks);
		else if(servo.type == ServoStatus::Type::REVOLUTE)
			pos = ticksToRad(ticks);
		else
			ROS_BREAK();

		servo.position = servo.scaleToMetric * pos;
		servo.goal_pos = servo.position;
	}

	// Low pass filter
	double raw = (1.0 / 512.0) * (m_status.air_pressure - 512);
	double alpha = m_param_airVel_alpha();
	double alpha_lower = m_param_airVel_alpha_lower();
	m_airVel = alpha * raw + (1.0 - alpha) * m_airVel;
	m_airVelLowPass = alpha_lower *
		raw + (1.0 - alpha_lower) * m_airVelLowPass;

	if(!m_status.vacuum_on)
		m_lastOffTime = ros::Time::now();

	if((ros::Time::now() - m_lastOffTime) < ros::Duration(m_param_airVel_minTime()))
		m_airVelLowPass = 1.0;

	// High-level filter
	bool currentlySomethingOnTip = m_status.vacuum_on && m_airVelLowPass < m_param_airVel_onTipThreshold();

	m_somethingOnTip = currentlySomethingOnTip
		&& (ros::Time::now() - m_lastOffTime) > ros::Duration(m_param_airVel_minTime());

	bool currentlyWellAttached =
		m_status.vacuum_on && m_airVel < m_param_airVel_wellAttachedThreshold();

	m_objectWellAttached = currentlyWellAttached
		&& (ros::Time::now() - m_lastOffTime) > ros::Duration(m_param_airVel_minTime());

	ControllerState state = {};
	state.air_velocity  = m_airVel;
	state.air_velocity_low_pass  = m_airVelLowPass;
	state.light_duty    = m_status.light_duty;
	state.vacuum_active = m_status.vacuum_on;
	state.vacuum_power_active = m_status.vacuum_power_on;
	state.something_on_tip = m_somethingOnTip;
	state.object_well_attached = m_objectWellAttached;
	m_pub_status.publish(state);

	plot_msgs::Plot plot;
	plot.header.stamp = ros::Time::now();

	plot.points.resize(7);
	plot.points[0].name = "/apc_interface/air_velocity_low_pass";
	plot.points[0].value = m_airVelLowPass;
	plot.points[1].name = "/apc_interface/air_velocity";
	plot.points[1].value = m_airVel;
	plot.points[2].name = "/apc_interface/air_velocity_raw";
	plot.points[2].value = raw;
	plot.points[3].name = "/apc_interface/vacuum_on";
	plot.points[3].value = m_status.vacuum_on;
	plot.points[4].name = "/apc_interface/vacuum_power_on";
	plot.points[4].value = m_status.vacuum_power_on;
	plot.points[5].name = "/apc_interface/something_on_tip";
	plot.points[5].value = m_somethingOnTip;
	plot.points[6].name  = "/apc_interface/object_well_attached";
	plot.points[6].value = m_objectWellAttached;

	m_pub_plot.publish(plot);

#if 0
	if(!m_logFile)
	{
		m_logFile = fopen("/tmp/apc_interface_log.txt", "w");
		if(!m_logFile)
		{
			ROS_ERROR("Could not opan log file!");
			return;
		}
		fprintf(m_logFile, "%s %s %s %s %s %s %s\n",
			"time_stamp", "air_vel_low_pass", "air_vel",
			"air_vel_raw", "vacuum", "something_on_tip",
			"object_well_attached"
		);
	}

	fprintf(m_logFile, "%3f %3f %3f %3f %d %d %d\n",
		ros::Time::now().toSec(),
		m_airVelLowPass, m_airVel, raw, m_status.vacuum_on,
		m_somethingOnTip, m_objectWellAttached
	);
#endif
}

void APCInterface::initMultiTurnOffset(const std::string& name)
{
	ServoStatus* servo = nullptr;
	for(auto& s : m_status.servos)
		if(s.name == name)
			servo = &s;

	if(!servo)
	{
		ROS_ERROR("Could not find servo with name: %s", name.c_str());
		return;
	}

	m_ctrl.updateStatus(&m_status);
	for(auto& servo : m_status.servos)
		servo.goal_position_ticks = servo.position_ticks;


	auto saved_torque = servo->max_torque;
	servo->max_torque = 200;
	servo->goal_position_ticks = -28672;
	m_ctrl.sendMaxTorque(m_status);
	m_ctrl.sendPositionCmds(m_status);

	ros::Duration maxDuration(30.0);
	ros::Duration holdDuration(0);
	ros::Time startTime = ros::Time::now();
	ros::Time loopTime = ros::Time::now();
	ros::Rate rate(50.0);
	int16_t oldPosition = INT16_MAX;
	auto maxHoldDuration = ros::Duration(m_param_multiTurnInitTime());
	auto threshold = m_param_multiTurnInitThreshold();
	while(ros::Time::now() - startTime < maxDuration)
	{
		m_ctrl.updateStatus(&m_status);
		if(abs(servo->position_ticks - oldPosition) < threshold)
		{
			ROS_INFO("Holding Position for %f", holdDuration.toSec());
			ROS_INFO("Current difference to servo is %d", servo->position_ticks-oldPosition);
			holdDuration += ros::Time::now() - loopTime;
			if(holdDuration > maxHoldDuration)
			{
				ROS_INFO("Position seems stable. Break.");
				break;
			}
		}
		else
			holdDuration = ros::Duration(0);
		oldPosition = servo->position_ticks;
		loopTime = ros::Time::now();
		rate.sleep();
	}

	m_ctrl.updateStatus(&m_status);
	servo->goal_position_offset = servo->position_ticks;
	ROS_INFO("final measured position: %d => offset %d", servo->position_ticks, servo->goal_position_offset);
	servo->goal_position_ticks = servo->goal_position_offset;
	servo->max_torque = saved_torque;
	m_ctrl.sendPositionCmds(m_status);
	m_ctrl.sendMaxTorque(m_status);
}


void APCInterface::shutdownServos()
{
	for(auto& servo : m_status.servos)
		servo.torque_enabled = false;
	m_ctrl.sendTorqueEnable(m_status);
}


bool APCInterface::srv_rebootController(
	std_srvs::EmptyRequest&,
	std_srvs::EmptyResponse&
)
{
	std::lock_guard<std::mutex> lock(m_ctrl_lock);
	return m_ctrl.rebootController();
}

bool APCInterface::srv_switchVacuum(
	std_srvs::SetBoolRequest& req,
	std_srvs::SetBoolResponse& res
)
{
	std::lock_guard<std::mutex> lock(m_ctrl_lock);
	res.success = m_ctrl.switchVacuum(req.data);
	return res.success;
}

bool APCInterface::srv_switchVacuumPower(
	std_srvs::SetBoolRequest& req,
	std_srvs::SetBoolResponse& res
)
{
	std::lock_guard<std::mutex> lock(m_ctrl_lock);
	res.success = m_ctrl.switchVacuumPower(req.data);
	return res.success;
}

bool APCInterface::srv_dimLight(
	apc_interface::DimLightRequest& req,
	apc_interface::DimLightResponse& res
)
{
	std::lock_guard<std::mutex> lock(m_ctrl_lock);
	return m_ctrl.dimLEDs(req.duty);
}

bool APCInterface::srv_setSuctionStrength(
	apc_interface::SuctionStrengthRequest& req,
	apc_interface::SuctionStrengthResponse& res
)
{
	std::lock_guard<std::mutex> lock(m_ctrl_lock);
	auto strength = req.strength;
	strength = std::min(strength, 1.0f);
	strength = std::max(strength, 0.0f);
	// NOTE (sebastian): release opening is defined as
	// 1 - [desired strength]
	m_releaseOpening = 1.0f - strength;

	return true;
}

bool APCInterface::srv_enableComm(
	std_srvs::SetBoolRequest& req,
	std_srvs::SetBoolResponse& res
)
{
	m_commEnabled = req.data;
	return true;
}

APCInterface::APCInterface()
 : m_nh("~")
 , m_param_airVel_alpha("/apc_interface/airVel/alpha", 0.0, 0.0001, 1.0, 0.5)
 , m_param_airVel_alpha_lower("/apc_interface/airVel/alpha_extra_low_pass", 0.0, 0.0001, 1.5, 0.7)
 , m_param_airVel_onTipThreshold("/apc_interface/airVel/something_on_tip", 0.0, 0.0001, 1.0, 0.1)
 , m_param_airVel_wellAttachedThreshold("/apc_interface/airVel/well_attached", 0.0, 0.0001, 1.0, 0.1)
 , m_param_airVel_minTime("/apc_interface/airVel/minTime", 0.0, 0.01, 10.0, 1.7)
 , m_param_multiTurnInitThreshold("/apc_interface/multi_turn_init_threshold", 1, 1, 20, 2)
 , m_param_multiTurnInitTime("/apc_interface/multi_turn_init_time", 0.1f, 0.05f, 0.5f, 0.2f)
{
	if(!m_ctrl.initUART("/dev/apc_controller"))
		throw std::runtime_error("Can't connect to controller. Abort.");
	if(!m_ctrl.connect())
		throw std::runtime_error("Different Protocol Versions detected. Abort!");

	m_srv_rebootController = m_nh.advertiseService(
		"/apc_interface/reboot_controller",
		&APCInterface::srv_rebootController, this
	);

	m_srv_switchVacuum = m_nh.advertiseService(
		"/apc_interface/switch_vacuum",
		&APCInterface::srv_switchVacuum, this
	);

	m_srv_switchVacuumPower = m_nh.advertiseService(
		"/apc_interface/switch_vacuum_power",
		&APCInterface::srv_switchVacuumPower, this
	);


	m_srv_dimLight = m_nh.advertiseService(
		"/apc_interface/dim",
		&APCInterface::srv_dimLight, this
	);

	m_srv_suctionStrength = m_nh.advertiseService(
		"/apc_interface/set_suction_strength",
		&APCInterface::srv_setSuctionStrength, this
	);

	m_srv_enableComm = m_nh.advertiseService(
		"/apc_interface/enable_servo_comm",
		&APCInterface::srv_enableComm, this
	);


	m_pub_status = m_nh.advertise<apc_interface::ControllerState>(
		"/apc_interface/controller_state", 1
	);

	m_pub_plot = m_nh.advertise<plot_msgs::Plot>(
		"/plot", 50
	);

	m_pub_cmdPlot = m_nh.advertise<plot_msgs::JointCommand>(
		"/joint_commands", 50
	);
}


}

int main(int argc, char** argv)
{
	// Get real-time priority
	sched_param schedparm;
	memset(&schedparm, 0, sizeof(schedparm));
	schedparm.sched_priority = 1;
	if(pthread_setschedparam(pthread_self(), SCHED_FIFO, &schedparm) != 0)
	{
		ROS_ERROR("Could not get realtime priority");
		ROS_ERROR("I'm going to run without realtime priority!");
	}

	// Set I/O priority to real-time
	ioprio_set(IOPRIO_WHO_PROCESS, getpid(), (IOPRIO_CLASS_RT << 13) | 0);
	ros::init(argc, argv, "apc_interface");
	apc_interface::APCInterface i;

	// Servo definition
	std::vector<apc_interface::ServoStatus> s;
	{
		s.resize(ProtoConstants::NUM_SERVOS);
		s[0].id = 1;
		s[0].name = "suc_finger_joint";
		s[0].type = apc_interface::ServoStatus::Type::LINEAR;
		s[0].max_torque = 1023;
		s[1].id = 2;
		s[1].name = "suc_fingertip_joint";
		s[1].goal_position_offset = 2030;
		s[1].max_torque = 50; // FIXME: Magic value, needs to be synced with EEPROM value
		s[1].scaleToMetric = -90.0 / 94.04;
		s[1].type = apc_interface::ServoStatus::Type::REVOLUTE;
		s[1].max_torque = 300;
		s[2].id = 3;
		s[2].name = releaseServoName;
		s[2].type = apc_interface::ServoStatus::Type::REVOLUTE;
		s[2].goal_position_offset = 2057;
		s[2].scaleToMetric = 1.0 / ((2816 - 2057) * radPerTick);
		s[2].max_torque = 1023;
	}
	i.setupServos(std::move(s));


	ROS_INFO("Calibrating the finger...");
	i.initMultiTurnOffset("suc_finger_joint");
	ROS_INFO("Finger calibration finished.");

	controller_manager::ControllerManager cm(&i);

	ros::AsyncSpinner spinner(1);
	spinner.start();

	auto t = ros::Time::now();
	ros::Rate rate(ros::Duration(0.02));

	while(ros::ok())
	{
		i.readFeedback();
		auto t_new = ros::Time::now();
		cm.update(t_new, t_new - t);
		t = t_new;
		i.writePos();

		rate.sleep();
	}

	i.shutdownServos();

	return 0;
}
//TODO handle estop
