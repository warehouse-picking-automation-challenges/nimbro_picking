// Talk directly to a Momaro microcontroller.
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "drc_proto.h"
#include "../protocol/drc_proto_constants.h"

#include "controller_tool_dxl.h"
#include "controller_tool_hand.h"

#include <boost/program_options.hpp>

#include <sys/fcntl.h>
#include <sys/select.h>

#include <fcntl.h>
#include <termios.h>

#include <libucomm/envelope.h>
#include <libucomm/checksum.h>

#include <iostream>
#include <stdexcept>

namespace po = boost::program_options;

int g_fd = -1;

class FDWriter
{
public:
	bool writeChar(uint8_t c)
	{
		printf("TX: %02X\n", c);
		if(write(g_fd, &c, 1) != 1)
		{
			perror("Could not write");
			throw std::runtime_error("Could not write");
		}
		return true;
	}
};

typedef uc::EnvelopeWriter<uc::InvertedModSumGenerator, FDWriter> EnvelopeWriter;
typedef uc::IO<EnvelopeWriter, uc::IO_W> SimpleWriter;
typedef Proto<SimpleWriter> WProto;

typedef uc::EnvelopeReader<uc::InvertedModSumGenerator, 1024> EnvelopeReader;
typedef uc::IO<EnvelopeReader, uc::IO_R> SimpleReader;
typedef Proto<SimpleReader> RProto;

EnvelopeReader g_reader;
FDWriter g_output;
EnvelopeWriter g_writer(&g_output);



bool readMsg(uint64_t timeout_usec = 100000UL)
{
	timeval timeout;
	timeout.tv_sec = timeout_usec / 1000000;
	timeout.tv_usec = timeout_usec % 1000000;

	unsigned int rxSize = 0;
	uint8_t rxBuf[256];

	printf("RX:");

	while(1)
	{
		unsigned int i;

		for(i = 0; i < rxSize; ++i)
		{
			printf(" %02X", rxBuf[i]);
			if(g_reader.take(rxBuf[i]) == EnvelopeReader::NEW_MESSAGE)
			{
				rxSize = rxSize - 1 - i;
				memmove(&rxBuf[0], &rxBuf[i+1], rxSize);

				printf("\n");
				return true;
			}
		}
		fflush(stdout);

		fd_set fds;
		FD_ZERO(&fds);
		FD_SET(g_fd, &fds);

		int ret = select(g_fd+1, &fds, 0, 0, &timeout);
		if(ret < 0)
		{
			if(errno == EAGAIN || errno == EINTR)
				continue;

			perror("select() fail");
			return false;
		}

		if(ret == 0 || !FD_ISSET(g_fd, &fds))
		{
			fprintf(stderr, "timeout\n");
			return false; // timeout
		}

		ret = read(g_fd, rxBuf, sizeof(rxBuf));
		if(ret < 0)
		{
			perror("read() fail");
			return false;
		}
		if(ret == 0)
		{
			throw std::runtime_error("Got size 0 from read()");
		}

		rxSize = ret;

#if COMM_DEBUG
		printf("RX:");
		for(unsigned int i = 0; i < rxSize; ++i)
			printf(" %02x", rxBuf[i]);
		printf("\n");
#endif
	}
}

bool connect()
{
	for(int tries = 0; tries < 5; ++tries)
	{
		WProto::ConnectMsg msg;
		g_writer << msg;

		if(!readMsg())
			continue;

		if(g_reader.msgCode() != RProto::ConnectMsgReply::MSG_CODE)
		{
			fprintf(stderr,
				"Unexpected msg with code %d while trying to connect...\n",
				g_reader.msgCode()
			);
			continue;
		}

		RProto::ConnectMsgReply reply;
		reply.version = 255;
		g_reader >> reply;

		if(reply.version == PROTOCOL_VERSION)
		{
			if(reply.error & CONNECT_MSG_ERROR_FLAG_NOT_SHUT_DOWN)
			{
				fprintf(stderr, "µC is in safety mode (was not faded out)\n");
				return false;
			}
			else if(reply.error == 0)
			{
				printf("Connected to controller (firmware version %u)\n", reply.version);
				return true;
			}
		}
		else
		{
			fprintf(stderr, "Firmware version differs from the one I'm built for: µC: %u, me: %u\n", reply.version, PROTOCOL_VERSION);
			return false;
		}
	}

	fprintf(stderr, "No answer to ConnectMsg packet!\n");
	return false;
}

void enterDXLPassthrough()
{
	// Enter passthrough mode
	WProto::StartPassthroughMsg startMsg;
	g_writer << startMsg;

	if(!readMsg())
		throw std::runtime_error("Could not enter passthrough mode");

	if(g_reader.msgCode() != RProto::StartPassthroughReply::MSG_CODE)
		throw std::runtime_error("Received invalid reply to start passthrough command");

	RProto::StartPassthroughReply reply;
	memset(&reply, 0, sizeof(reply));
	g_reader >> reply;
}

void exitDXLPassthrough()
{
	RProto::StartPassthroughReply reply;
	memset(&reply, 0, sizeof(reply));

	// Exit passthrough
	if(write(g_fd, PASSTHROUGH_EXIT_KEY, sizeof(PASSTHROUGH_EXIT_KEY)) != sizeof(PASSTHROUGH_EXIT_KEY))
	{
		perror("Could not write");
		throw std::runtime_error("Could not write");
	}

	if(!readMsg())
		throw std::runtime_error("Could not exit passthrough mode");

	if(g_reader.msgCode() != RProto::StartPassthroughReply::MSG_CODE)
		throw std::runtime_error("Received invalid reply to stop passthrough command");

	reply.active = true;
	g_reader >> reply;

	if(reply.active)
		throw std::runtime_error("Could not exit passthrough mode (active == true)");
}

void dxlShell()
{
	enterDXLPassthrough();

	printf("Entered passthrough mode. Launching interactive shell\n");

	dxl::shell();

	sleep(1);
	tcflush(g_fd, TCIFLUSH);

	exitDXLPassthrough();

	printf("Exited passthrough mode\n");
}

int handFlash(const std::string& binary)
{
	enterDXLPassthrough();

	printf("Entered passthrough mode.\n");

	int ret = hand::flash(binary);

	sleep(1);
	tcflush(g_fd, TCIFLUSH);

	exitDXLPassthrough();

	printf("Exited passthrough mode\n");

	if(ret == 0)
		printf("============== SUCCESS ===============");
	else
		printf("============== FAILURE ===============");

	return ret;
}

void reset()
{
	WProto::RebootControllerMsg msg;
	msg.key1 = REBOOT_KEY1;
	msg.key2 = REBOOT_KEY2;
	g_writer << msg;

	readMsg();
}

int main(int argc, char** argv)
{
	po::options_description desc("Allowed options");
	desc.add_options()
		("help", "This help message")
		("device", po::value<std::string>()->required(), "Device file of the µC")
	;

	po::options_description cmds("Commands");
	cmds.add_options()
		("dxl", "Enter interactive DXL shell")
		("raw-dxl", "Directly talk to a DXL 2.0 bus")
		("reset", "Reset the controller")
		("flash-hand", po::value<std::string>(), "Flash the hand controller")
		("raw-flash-hand", po::value<std::string>(), "Flash the hand controller")
		("exit-dxl", "Exit DXL passthrough")
	;

	desc.add(cmds);

	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);
	po::notify(vm);

	if(vm.count("help"))
	{
		std::cout << desc << "\n";
		return 1;
	}

	std::string filename = vm["device"].as<std::string>();
	g_fd = open(filename.c_str(), O_RDWR);

	if(g_fd < 0)
	{
		fprintf(stderr, "Could not open device '%s': %s\n",
			filename.c_str(), strerror(errno)
		);
		return 1;
	}

	// Setup terminal
	// Lock the serial connection
	if(lockf(g_fd, F_TLOCK, 0) != 0)
	{
		perror("Could not acquire serial lock (is robotcontrol running?)");
		return 1;
	}

	// Initialise the terminal interface struct
	struct termios config;
	memset(&config, 0, sizeof(config));
	if(tcgetattr(g_fd, &config) != 0)
	{
		perror("Could not get terminal attributes");
		return 1;
	}

	cfmakeraw(&config);
	cfsetspeed(&config, B115200);

	// Set the required terminal attributes
	if(tcsetattr(g_fd, TCSANOW, &config) != 0)
	{
		perror("Could not set terminal attributes");
		return 1;
	}

	if(vm.count("raw-dxl"))
	{
		dxl::shell();
		return 0;
	}

	if(vm.count("raw-flash-hand"))
	{
		hand::flash(vm["raw-flash-hand"].as<std::string>());
		return 0;
	}

	if(vm.count("reset"))
	{
		reset();
		return 0;
	}

	if(vm.count("exit-dxl"))
	{
		exitDXLPassthrough();
		return 0;
	}

	if(!connect())
	{
		fprintf(stderr, "Could not connect to microcontroller.\n");
		return 1;
	}

	if(vm.count("dxl"))
		dxlShell();


	if(vm.count("flash-hand"))
		handFlash(vm["flash-hand"].as<std::string>());

	return 0;
}
