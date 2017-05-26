// Commandline utility to communicate with the controller
// Author: Sebastian Schüller<schuell1@cs.uni-bonn.de>


#include <apc_interface/apc_controller.h>

#include <boost/program_options.hpp>

#include <ros/console.h>

int main(int argc, char** argv)
{
	namespace po=boost::program_options;
	po::options_description desc("Allowed options");
	desc.add_options()
		("help,h", "This help message")
		("device,d", po::value<std::string>(), "Device file of the µC")
	;

	po::options_description cmds("Commands");
	cmds.add_options()
		("reset,r", "Reset the controller")
		("status,s", "Get single status packet from controller")
		("vacuum,v", po::value<bool>(), "Turn Vacuum On/Off")
		("lightdim,l", po::value<unsigned int>(), "Dim SR300 LED Strips")
		("id", po::value<uint16_t>(), "Id to command servo")
		("position,p", po::value<uint16_t>(), "Position command to send to servo with id '--id'")
		("torque-en,t", po::value<bool>(), "Enable/Disable torque for servo with id '--id'")
	;

	desc.add(cmds);

	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);
	po::notify(vm);

	if(vm.count("help"))
	{
		ROS_INFO_STREAM(desc);
		return 1;
	}

	if(!vm.count("device"))
	{
		ROS_ERROR("I Need a device specified to do anything.");
		ROS_INFO_STREAM(desc);
		return 1;
	}

	apc_interface::ControllerStatus status = {};
	status.servos.resize(3);
	status.servos[0].id                   = 1;
	status.servos[0].name                 = "suc_finger_joint";
	status.servos[0].type                 = apc_interface::ServoStatus::Type::LINEAR;
	status.servos[0].max_torque           = 500;
	status.servos[1].id                   = 2;
	status.servos[1].name                 = "suc_fingertip_joint";
	status.servos[1].goal_position_offset = 1815;
	status.servos[1].max_torque           = 50; // FIXME: Magic value, needs to be synced with EEPROM value
	status.servos[1].scaleToMetric        = -90.0 / 101.51;
	status.servos[1].type                 = apc_interface::ServoStatus::Type::REVOLUTE;
	status.servos[2].id                   = 3;
	status.servos[2].name                 = "joint";
	status.servos[2].goal_position_offset = 1815;
	status.servos[2].max_torque           = 50; // FIXME: Magic value, needs to be synced with EEPROM value
	status.servos[2].scaleToMetric        = -90.0 / 101.51;
	status.servos[2].type                 = apc_interface::ServoStatus::Type::REVOLUTE;
	std::string device = vm["device"].as<std::string>();
	apc_interface::Controller c;
	if(!c.initUART(device))
	{
		ROS_ERROR("Could not connect to controller");
		return 1;
	}

	if(vm.count("reset"))
	{
		c.rebootController();
		return 0;
	}

	if(!c.connect())
	{
		return 1;
	}



	if(!c.initializeServos({1,2,3}))
	{
		ROS_ERROR("Could not init servos!");
		return 1;
	}
// 	c.updateEEPROM(&status);
// 	c.sendMaxTorque(status);



	if(vm.count("lightdim"))
	{
		unsigned int duty = vm["lightdim"].as<unsigned int>();
		if(duty > UINT8_MAX)
		{
			ROS_WARN("Maximal value for dimming is %d.", UINT8_MAX);
			ROS_WARN("Will truncate value.");
			duty = UINT8_MAX;
		}
		if(!c.dimLEDs(duty))
			ROS_ERROR("Failed to set light duty!");
		else
			ROS_INFO("Updated light duty");
	}
	if(vm.count("vacuum"))
	{
		c.switchVacuum(vm["vacuum"].as<bool>());
	}

	if(vm.count("torque-en"))
	{
		if(!vm.count("id"))
		{
			ROS_ERROR("Need servo id to enable torque");
			return 1;
		}
		//FIXME(sebastian): Un-Break torque-enable cmd

		//uint16_t id = vm["id"].as<uint16_t>();
		//bool e = vm["torque-en"].as<bool>();
		//uint8_t enable = e ? 1 : 0;
		//if(!c.sendTorqueEnable({{id, enable}}))
		//{
			//ROS_ERROR("Sending position cmd failed!");
		//}
		
	}

	if(vm.count("position"))
	{
		if(!vm.count("id"))
		{
			ROS_ERROR("Need servo id to write a position to");
			return 1;
		}
		if(!c.updateStatus(&status))
		{
			ROS_ERROR("Could not get status. Won't set new positions");
			return 1;
		}

		status.servos[0].goal_position_ticks =
			status.servos[0].position_ticks;
		status.servos[1].goal_position_ticks =
			status.servos[1].position_ticks;

		for(auto& servo : status.servos)
		{
			if(servo.id == vm["id"].as<uint16_t>())
				servo.goal_position_ticks =
					vm["position"].as<uint16_t>();
		}

		if(!c.sendPositionCmds(status))
		{
			ROS_ERROR("Failed to send position commands");
		}

	}

	if(vm.count("status"))
	{
		usleep(3000);

		apc_interface::ControllerDebugStatus dbg = {};
		c.updateStatus(&status);
		c.getDbgStatus(&dbg);
		ROS_INFO("Controller Status");
		ROS_INFO("Status Flags: 0x%02x", status.status);
		ROS_INFO("Vacuum      : %s", status.vacuum_on ? "ON" : "OFF");
		ROS_INFO("Light Duty  : %d", status.light_duty);
		ROS_INFO("Air Pressure: %d", status.air_pressure);
		ROS_INFO("Timeouts    : %d", status.timeouts);

		ROS_INFO("Servo Status");
		ROS_INFO("Num Servos  : %lu\n", status.servos.size());
		for(const auto& servo : status.servos)
		{
			ROS_INFO("    Id          : %d", servo.id);
			ROS_INFO("    Flags       : %d", servo.flags);
			ROS_INFO("    Goal Pos    : %d", servo.goal_position_ticks);
			ROS_INFO("    Goal Offset : %d", servo.goal_position_offset);
			ROS_INFO("    Pos         : %d", servo.position_ticks);
			ROS_INFO("    Speed       : %d", servo.velocity_ticks);
			ROS_INFO("    Load        : %d", servo.load);
			ROS_INFO("    Voltage     : %d", servo.voltage);
			ROS_INFO("    Temperature : %d", servo.temperature);
			ROS_INFO("    Error       : %d\n", servo.error);
		}

		ROS_INFO("Debug Information");
		if(dbg.comm_statuscheck_uninitialized)
			ROS_WARN("Tried to get status information from uninitialized servos!");
		if(dbg.comm_poscmd_uninitialized)
			ROS_WARN("Tried to write position commands to  uninitialized servos!");
		if(dbg.comm_poscmd_invld_id)
			ROS_WARN("Tried to write position commands to servo with invalid id!");
		if(dbg.comm_trqen_uninitialized)
			ROS_WARN("Tried to enable torque on uninitialized servos!");
		if(dbg.comm_trqen_invld_id)
			ROS_WARN("Tried to enable torque on servo with invalid id!");
		if(dbg.dxl_bulkread_invld_id)
			ROS_WARN("Bulkread response contained an invalid servo id!");
		if(dbg.dxl_bulkread_invld_size)
			ROS_WARN("Bulkread response contained an invalid number of parameters!");


		return 0;
	}

	usleep(3000);
	return 0;
}
