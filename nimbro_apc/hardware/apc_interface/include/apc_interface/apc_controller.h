// Communication controller for APC
// Author: Sebastian Schüller<schuell1@cs.uni-bonn.de>


#ifndef APC_CONTROLLER_H
#define APC_CONTROLLER_H

#include <string>
#include <vector>
#include <memory>

#include "controller_types.h"


struct UARTBuffer;

namespace apc_interface
{

class Controller
{
public:
	Controller();
	~Controller();
	bool initUART(std::string filename);
	bool connect();
	bool initializeServos(std::vector<uint16_t> ids);
	bool initializeServoOffset(uint16_t id);

	bool rebootController();
	bool dimLEDs(uint8_t duty);
	bool switchVacuum(bool on);
	bool switchVacuumPower(bool on);
	bool sendPositionCmds(const ControllerStatus& status);
	bool sendTorqueEnable(const ControllerStatus& status);
	bool sendMaxTorque(const ControllerStatus& status);

	bool updateStatus(ControllerStatus* status);
	bool updateEEPROM(ControllerStatus* status);
	bool getDbgStatus(ControllerDebugStatus* status);


private:
	int m_fd;

	bool sendBuffer();
	bool readMsg(unsigned int timeout_usec = 1000000);
	std::unique_ptr<UARTBuffer> m_buffer;
};
}

#endif
