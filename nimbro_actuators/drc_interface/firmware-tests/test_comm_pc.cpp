// Unit tests for comm_pc module
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "test_support.h"
#include <random>
#include <iostream>

std::random_device s_rdev;
std::mt19937 s_rgen(s_rdev());
std::uniform_int_distribution<int> s_rdistr(1,100);

bool use_fuzzy = false;

struct test_servo
{
	enum mode
	{
		POS = 0,
		VEL
	};

	uint16_t id;
	mode mod;
	uint16_t max_velocity = 1000;
	bool in_use;
	bool should_fade;
	test_servo(uint8_t id, mode mod, bool in_use = false, bool should_fade = true)
	 : id(id)
	 , mod(mod)
	 , in_use(in_use)
	 , should_fade(should_fade)
	{};
};


static std::vector<test_servo> s_test_servos = {
	test_servo( 1, test_servo::POS, true,  true ),
	test_servo( 2, test_servo::POS, true,  true ),
	test_servo( 3, test_servo::POS, true,  true ),
	test_servo( 4, test_servo::POS, true,  true ),
	test_servo( 5, test_servo::POS, true,  true ),
	test_servo( 6, test_servo::POS, true,  true ),
	test_servo( 7, test_servo::POS, true,  true ),
	test_servo( 8, test_servo::POS, true,  true )
};

bool fuzzy_bool(uint8_t chance)
{
	return s_rdistr(s_rgen) < chance ? true : false;
}

uint16_t random_uint16_t(const uint16_t low_cap,const uint16_t high_cap)
{
	std::uniform_int_distribution<uint16_t> rdistr(low_cap, high_cap);
	return rdistr(s_rgen);
}

void connect(avr_t* avr, PCUART* pc)
{
	// Let the AVR boot up and set up the UART
	stepAVR(avr, 50000);

	// Write initial connection packet
	WProto::ConnectMsg msg;
	EnvelopeWriter pc_to_uc(pc);
	pc_to_uc << msg;

	bool connectReplyReceived = false;
	pc->setMsgCallback([&](EnvelopeReader* reader) {
		REQUIRE(reader->msgCode() == RProto::ConnectMsgReply::MSG_CODE);

		RProto::ConnectMsgReply reply;
		REQUIRE(reader->read(&reply));

		CHECK(reply.version == PROTOCOL_VERSION);
		CHECK(reply.error == 0);
		printf("Connected to µC with version %d\n", reply.version);
		connectReplyReceived = true;
	});

	stepAVR(avr, 50000);

	REQUIRE(connectReplyReceived);
}

void configure(avr_t* avr, PCUART* pc, DXLUART* dxl, uint8_t expected_errors = 0)
{
	// Write configuration packet
	WProto::ServoConfigurationMsg msg;
	msg.ttl_bus_address = 29;
	msg.ttl_bus_base_id = 10;

	std::vector<WProto::ServoConfiguration> config;

	for (auto ts : s_test_servos)
	{
		if(!ts.in_use) continue;

		WProto::ServoConfiguration servo;
		servo.id = ts.id;
		servo.max_velocity = ts.max_velocity;
		servo.fadein_velocity = 1000;
		servo.flags = 0;
		if (ts.mod == test_servo::VEL) servo.flags |= SERVO_FLAG_VELOCITY_CONTROL;

		config.push_back(servo);
	}


	msg.servos.setData(config.data(), config.size());

	EnvelopeWriter pc_to_uc(pc);
	pc_to_uc << msg;

	bool replyReceived = false;
	pc->setMsgCallback([&](EnvelopeReader* reader) {
		REQUIRE(reader->msgCode() == RProto::ServoConfiguredMsg::MSG_CODE);

		RProto::ServoConfiguredMsg reply;
		REQUIRE(reader->read(&reply));
		CHECK(reply.error == expected_errors);

		replyReceived = true;
	});

	stepAVR(avr, 10*1000000);

	REQUIRE(replyReceived);

	// Check that the LED is blue on all configured servos
	for(auto ts : s_test_servos)
	{
		if(!ts.in_use) continue;

		uint16_t id = ts.id;

		INFO("LED should be blue on servo with ID " << id);

		REQUIRE(dxl->dxlRegister(id, 563, 1) == 0);    // R
		REQUIRE(dxl->dxlRegister(id, 564, 1) == 0);    // G
		REQUIRE(dxl->dxlRegister(id, 565, 1) == 255);  // B
	}

	// Check that the LED is not blue on all unconfigured servos
	for(auto ts : s_test_servos)
	{
		if(ts.in_use) continue;

		uint16_t id = ts.id;

		INFO("LED should not be blue on servo with ID " << id);

		REQUIRE(dxl->dxlRegister(id, 563, 1) == 0);    // R
		REQUIRE(dxl->dxlRegister(id, 564, 1) == 0);    // G
		REQUIRE(dxl->dxlRegister(id, 565, 1) == 0);    // B
	}

	// Check mode flags
	for(auto ts : s_test_servos)
	{
		if(!ts.in_use) continue;

		uint16_t id = ts.id;
		uint8_t mode = (ts.mod == test_servo::VEL) ? 1 : 3;

		INFO("Servo " << id << " should have mode " << mode);
		REQUIRE(dxl->dxlRegister(id, 11, 1) == mode);
	}

	// Check internal Servo state
	WProto::IntrospectionMsg isMsg;
	pc_to_uc << isMsg;
	replyReceived = false;
	pc->setMsgCallback([&](EnvelopeReader* reader){
		REQUIRE(reader->msgCode() == RProto::IntrospectionReplyMsg::MSG_CODE);

		RProto::IntrospectionReplyMsg reply;
		REQUIRE(reader->read(&reply));
		CHECK((bool)reply.faded_after_connect == false);
		CHECK((reply.num_pro_servos + reply.num_ttl_servos) == s_test_servos.size());

		RProto::ServoIntrospection servo;
		while(reply.servos.next(&servo))
		{
			CHECK(servo.max_torque != 65335);
		}
	});
	stepAVR(avr, 1000000);
}

void fade_servo(std::vector<uint16_t> ids, uint32_t fade_time_ms, uint16_t torque, avr_t* avr, PCUART* pc, DXLUART* dxl)
{
	const unsigned int DXL_GOAL_TORQUE_ADDR = 604;
	
	WProto::FadeCommandMsg msg;

	std::vector<WProto::FadeCommand> fade;
	for(auto id : ids)
	{
		WProto::FadeCommand servo;
		servo.id = id;
		//servo.fadein_velocity = 2000;
		//servo.final_velocity = 10000;
		servo.torque = torque;

		fade.push_back(servo);
	}
	msg.fade_time = fade_time_ms;
	msg.servos.setData(fade.data(), fade.size());

	EnvelopeWriter pc_to_uc(pc);
	pc_to_uc << msg;

	bool replyReceived = false;
	pc->setMsgCallback([&](EnvelopeReader* reader) {
		REQUIRE(reader->msgCode() == RProto::FadeCommandMsgReply::MSG_CODE);

		RProto::FadeCommandMsgReply reply;
		REQUIRE(reader->read(&reply));
		REQUIRE(reply.error == 0);

		replyReceived = true;
	});

	for(int i = 0; !replyReceived && i != 40; ++i)
		stepAVR(avr, 16000000ULL / 10);

	REQUIRE(replyReceived);

	for(auto id : ids)
	{
		CAPTURE(id);
		uint16_t g_torque = torque == 0 ? 1 : torque;
		CHECK(dxl->dxlRegister(id, DXL_GOAL_TORQUE_ADDR, 2) == g_torque);
	}
}
/* Test won't work anymore because of a protocol update */
/*
TEST_CASE("weird_servo_fading", "[comm_pc]")
{
	const unsigned int DXL_TORQUE_ENABLE_ADDR = 562;
	const unsigned int DXL_GOAL_TORQUE_ADDR = 604;

	// Init avrsim
	avr_t* avr = initAVR();

	// Attach PC connection
	PCUART pc;
	pc.attach(avr, '0');

	// Attach DXL connection
	DXLUART dxl(21, 29);
	dxl.attach(avr, '3');

	connect(avr, &pc);

	// Set up fading dxl register
	for(uint8_t id = 1; id < 21; ++id)
	{
		dxl.setDXLRegister(id, DXL_TORQUE_ENABLE_ADDR, 1, 1);
		REQUIRE(dxl.dxlRegister(id, DXL_TORQUE_ENABLE_ADDR, 1) == 1);
		dxl.setDXLRegister(id, DXL_GOAL_TORQUE_ADDR, 2, 0x7FFF);
		REQUIRE(dxl.dxlRegister(id, DXL_GOAL_TORQUE_ADDR, 2) == 0x7FFF);
	}

	// Custom configure
	std::vector<uint8_t> momaro_configure = {0xff, 0x02, 0x1d, 0x08, 0x10, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0b, 0x00, 0x00, 0x00, 0x02, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x02, 0x00, 0x08, 0x00, 0x00, 0x00, 0x02, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x02, 0x00, 0x0a, 0x00, 0x00, 0x00, 0x02, 0x00, 0x0e, 0x00, 0x00, 0x00, 0x02, 0x00, 0x09, 0x00, 0x00, 0x00, 0x02, 0x00, 0x0d, 0x00, 0x00, 0x00, 0x02, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xfd, 0x2c};

	for(auto c : momaro_configure)
		pc.writeChar(c);

	pc.setMsgCallback([&](EnvelopeReader* reader) {

	});

	stepAVR(avr, 1000000);

    // Custom Fade Out
    std::vector<uint8_t> l_elbow_fadeout = {0xff, 0x06, 0xf4, 0x01, 0x01, 0x04, 0xd0, 0x07, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0xff, 0xfd, 0xf1};
    for(auto c : l_elbow_fadeout)
        pc.writeChar(c);

    stepAVR(avr, 10000000ULL);
    
    const uint8_t L_ELB_ID = 4;
    CHECK(dxl.dxlRegister(L_ELB_ID, DXL_TORQUE_ENABLE_ADDR, 1) == 0);
    CHECK(dxl.dxlRegister(L_ELB_ID, DXL_GOAL_TORQUE_ADDR, 2) == 0x01);

    // Custom Fade in
    std::vector<uint8_t> l_elbow_fadein ={0xff, 0x06, 0xf4, 0x01, 0x01, 0x04, 0xd0, 0x07, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0xff, 0xfe, 0x7f, 0xff, 0xfd, 0x73};
    for(auto c : l_elbow_fadein)
        pc.writeChar(c);

    stepAVR(avr, 50000000ULL);
    
    CHECK(dxl.dxlRegister(L_ELB_ID, DXL_TORQUE_ENABLE_ADDR, 1) == 1);
    CHECK(dxl.dxlRegister(L_ELB_ID, DXL_GOAL_TORQUE_ADDR, 2) == 0x7FFF);

}*/


TEST_CASE("connect to hot", "[comm_pc]")
{
	const unsigned int DXL_TORQUE_ENABLE_ADDR = 562;
	const unsigned int DXL_GOAL_TORQUE_ADDR = 604;
	
	// Init avrsim
	avr_t* avr = initAVR();

	// Attach PC connection
	PCUART pc;
	pc.attach(avr, '0');

	// Attach DXL connection
	DXLUART dxl(s_test_servos.size(), 29);
	dxl.attach(avr, '3');

	// Set all torques in servos to 0
	for(auto ts : s_test_servos)
	{
		dxl.setDXLRegister(ts.id, DXL_TORQUE_ENABLE_ADDR, 1, 0);
		dxl.setDXLRegister(ts.id, DXL_GOAL_TORQUE_ADDR, 2, 0x1);
		REQUIRE(dxl.dxlRegister(ts.id, DXL_TORQUE_ENABLE_ADDR, 1) == 0);
		REQUIRE(dxl.dxlRegister(ts.id, DXL_GOAL_TORQUE_ADDR, 2) == 0x1);
	}

	// Connect and configure
	connect(avr, &pc);
	configure(avr, &pc, &dxl);

	// Fade in and possibly do stuff
	std::vector<uint16_t> fade_ids;
	for(auto ts : s_test_servos)
		if(ts.in_use && ts.should_fade) fade_ids.push_back(ts.id);
	fade_servo(fade_ids, 500, 32000, avr, &pc, &dxl);

	// Now our fictive Robotcontrol dies. And re-connects.
	// Custom connect msg to check error flags.
	WProto::ConnectMsg msg;
	EnvelopeWriter pc_to_uc(&pc);
	pc_to_uc << msg;

	bool connectReplyReceived = false;
	pc.setMsgCallback([&](EnvelopeReader* reader) {
		REQUIRE(reader->msgCode() == RProto::ConnectMsgReply::MSG_CODE);

		RProto::ConnectMsgReply reply;
		REQUIRE(reader->read(&reply));

		CHECK(reply.version == PROTOCOL_VERSION);
		CHECK(reply.error == CONNECT_MSG_ERROR_FLAG_NOT_SHUT_DOWN);
		printf("Connected to µC with version %d\n", reply.version);
		connectReplyReceived = true;
	});

	stepAVR(avr, 50000);

	REQUIRE(connectReplyReceived);
}

TEST_CASE("configure hot", "[comm_pc]")
{
	const unsigned int DXL_TORQUE_ENABLE_ADDR = 562;
	const unsigned int DXL_GOAL_TORQUE_ADDR = 604;
	
	// Init avrsim
	avr_t* avr = initAVR();

	// Attach PC connection
	PCUART pc;
	pc.attach(avr, '0');

	// Attach DXL connection
	DXLUART dxl(s_test_servos.size(), 29);
	dxl.attach(avr, '3');


	// Set initial torques high to simulate a hot configure
	for(auto ts : s_test_servos)
	{
		dxl.setDXLRegister(ts.id, DXL_TORQUE_ENABLE_ADDR, 1, 1);
		dxl.setDXLRegister(ts.id, DXL_GOAL_TORQUE_ADDR, 2, 0x7EEE);
		REQUIRE(dxl.dxlRegister(ts.id, DXL_TORQUE_ENABLE_ADDR, 1) == 1);
		REQUIRE(dxl.dxlRegister(ts.id, DXL_GOAL_TORQUE_ADDR, 2) == 0x7EEE);
	}

	// connect after 'reboot'
	connect(avr, &pc);
	configure(avr, &pc, &dxl, CONFIGURE_MSG_ERROR_FLAG_NOT_SHUT_DOWN);

	
}

TEST_CASE("connection", "[comm_pc]")
{

	if(use_fuzzy)
	{
		// Make the attributes of the tested servos a little bit random
		s_test_servos.clear();

		uint16_t max_servos = random_uint16_t(5,9);
		std::vector<uint16_t> servo_ids;
		for(auto i = 1; i <= max_servos; ++i) servo_ids.push_back(i);
		std::shuffle(servo_ids.begin(), servo_ids.end(), s_rgen);
		for(auto i = 0; i < max_servos; ++i)
		{
			test_servo::mode m;
			m = fuzzy_bool(85) ? test_servo::POS : test_servo::VEL;
			test_servo t(
				servo_ids[i],
				m,
				fuzzy_bool(85),
				fuzzy_bool(50)
			);
			s_test_servos.push_back(t);
		}


		std::cout << "\nTesting with " << max_servos << " servos.\n";
		std::cout << "Structure of tested servos:\n";
		for(auto ts : s_test_servos)
			std::cout << "    Id : " << ts.id
			<< ", Mode : " << (ts.mod == test_servo::POS ? "Position" : "Velocity")
			<< ", active : " << (ts.in_use ? "true" : "false")
			<< ", fading : " << (ts.should_fade ? "true" : "false")
			<< "\n";
		std::cout << "\n\n\n\n" << std::endl;

	}


	// Init avrsim
	avr_t* avr = initAVR();

	// Attach PC connection
	PCUART pc;
	pc.attach(avr, '0');

	// Attach DXL connection
	DXLUART dxl(s_test_servos.size(), 29);
	dxl.attach(avr, '3');




	connect(avr, &pc);
	configure(avr, &pc, &dxl);

	SECTION("commanding servos")
	{

		// The servos need to fade before accepting commands
		std::vector<uint16_t> init_servos;
		for(auto ts : s_test_servos)
			if(ts.in_use) init_servos.push_back(ts.id);

		fade_servo(init_servos, 50, 0x7FFF, avr, &pc, &dxl);

		EnvelopeWriter output(&pc);
		WProto::IntrospectionMsg isMsg;
		output << isMsg;

		pc.setMsgCallback([&](EnvelopeReader* reader){
			REQUIRE(reader->msgCode() == RProto::IntrospectionReplyMsg::MSG_CODE);
			RProto::IntrospectionReplyMsg reply;
			REQUIRE(reader->read(&reply));
			CHECK((bool)reply.faded_after_connect == true);
		});

		printf("Sending ServoCommandMsg\n");
		WProto::ServoCommandMsg msg;
		msg.request_id = 5050;

		std::vector<WProto::ServoCommand> servos;

		for(auto ts : s_test_servos)
		{
			if (!ts.in_use) continue;
			uint16_t id = ts.id;

			WProto::ServoCommand cmd;
			cmd.id = id;
			cmd.command = 10 * id + 5;
			servos.push_back(cmd);
		}

		msg.servos.setData(servos.data(), servos.size());

		// Setup some feedback in the servos that we will read
		for(auto ts : s_test_servos)
		{
			uint8_t id = (uint8_t)ts.id;

			dxl.setDXLRegister(id, 611, 4, 0x00FDFFFF + id - 2);
			dxl.setDXLRegister(id, 621, 2, 0x4F00 + id);
		}

		output << msg;

		bool replyReceived = false;

		// This lambda is executed as soon as we receive a packet
		pc.setMsgCallback([&](EnvelopeReader *reader){
			REQUIRE(reader->msgCode() == RProto::ServoFeedbackMsg::MSG_CODE);
			replyReceived = true;

			RProto::ServoFeedbackMsg reply;
			REQUIRE(reader->read(&reply));
			REQUIRE(reply.error == 0);
			REQUIRE(reply.request_id == msg.request_id);


			RProto::ServoFeedback servo;


			// Check that all used servos answered
			std::vector<bool> visited(s_test_servos.size(), true);
			for(unsigned int i = 0; i < visited.size(); ++i)
			{
				if(i >= s_test_servos.size()) break;
				auto* ts = &s_test_servos[i];
				if(ts->in_use) visited[i] = false;
			}

			auto* ts = &s_test_servos[0];
			unsigned int i = 0;
			while(reply.servos.next(&servo))
			{
				CAPTURE(i);
				REQUIRE(i < visited.size());
				REQUIRE(i < s_test_servos.size());
				// Scroll to the next used servo.
				while(!s_test_servos[i].in_use && i < s_test_servos.size()) ++i;
				REQUIRE(servo.id == s_test_servos[i].id);
				visited[i++] = true;


				INFO("Feedback for servo " << servo.id);
				CHECK(servo.packet_state == 0);
				CHECK(servo.position == 0x00FDFFFF + servo.id - 2);
				CHECK(servo.torque == 0x4F00 + servo.id);
			}

			for(unsigned int i = 0; i < sizeof(visited); ++i)
			{
				CAPTURE(i);
				CHECK(visited[i]);
			}

			REQUIRE(reply.error == 0);
		});

		// Let the AVR run for X cycles
		stepAVR(avr, 50*1000000);

		// Afterwards, we expect to have a reply
		REQUIRE(replyReceived);

		// Check that our commands got written correctly
		for(auto ts : s_test_servos)
		{
			if (!ts.in_use) continue;

			uint16_t id = ts.id;
			INFO("Checking id " << id << " commands");

			if(ts.mod == test_servo::VEL) // velocity control?
				CHECK(dxl.dxlRegister(id, 600, 4) == (10 * id + 5));
			else
				CHECK(dxl.dxlRegister(id, 596, 4) == (10 * id + 5));
		}
    }	

	SECTION("Fade")
	{

		std::vector<uint8_t> visitedIds = {};

		for (auto ts : s_test_servos)
			REQUIRE(dxl.dxlRegister(ts.id, 562, 1) == 0);

		for (auto ts : s_test_servos)
		{
			if(!ts.in_use || !ts.should_fade) continue;

			std::vector<uint16_t>ids;
			ids.push_back(ts.id);
			visitedIds.push_back(ts.id);
			fade_servo(ids, 500, 0x7FFF, avr, &pc, &dxl);

			for(auto tts : s_test_servos)
			{
				if(!tts.in_use) continue;
				if (std::find(visitedIds.begin(), visitedIds.end(), tts.id) != visitedIds.end())
					CHECK(dxl.dxlRegister(tts.id, 562, 1) == 1);
				else
					CHECK(dxl.dxlRegister(tts.id, 562, 1) == 0);
			}

		}

		for (auto ts : s_test_servos)
		{
			if(!ts.in_use || !ts.should_fade) continue;
			REQUIRE(dxl.dxlRegister(ts.id, 562, 1) == 1);
		}
		visitedIds.clear();

		for (auto ts : s_test_servos)
		{
			if(!ts.in_use || !ts.should_fade) continue;

			std::vector<uint16_t>ids;
			ids.push_back(ts.id);
			visitedIds.push_back(ts.id);
			fade_servo(ids, 500, 0, avr, &pc, &dxl);

			for(auto tts : s_test_servos)
			{
				if(!tts.in_use || !tts.should_fade) continue;
				if (std::find(visitedIds.begin(), visitedIds.end(), tts.id) != visitedIds.end())
					CHECK(dxl.dxlRegister(tts.id, 562, 1) == 0);
				else
					CHECK(dxl.dxlRegister(tts.id, 562, 1) == 1);
			}

		}
		for (auto ts : s_test_servos)
			REQUIRE(dxl.dxlRegister(ts.id, 562, 1) == 0);

	}
/*
	// Currently, hard E-Stop is not wired up.
	SECTION("E-Stop")
	{
		fadeIn(avr, &pc, &dxl);

		for(int id = 2; id <= 10; id += 2)
		{
			CHECK(dxl.dxlRegister(id, 562, 1) == 1);
		}

		printf("Enabling hard emergency stop\n");

		// Enable hard emergency stop
		avr_raise_irq(
			avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ('G'), 1),
			1
		);

		stepAVR(avr, 4 * 16000000ULL);

		for(int id = 2; id <= 10; id += 2)
		{
			CHECK(dxl.dxlRegister(id, 562, 1) == 0);
		}
	}
*/
}
