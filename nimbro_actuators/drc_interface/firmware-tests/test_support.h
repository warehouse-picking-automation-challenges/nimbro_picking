// Support routines for unit testing the firmware
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef TEST_SUPPORT_H
#define TEST_SUPPORT_H

#include <simavr/sim/sim_avr.h>
#include <simavr/sim/sim_elf.h>
#include <simavr/sim/sim_gdb.h>
#include <simavr/sim/sim_irq.h>
#include <simavr/sim/sim_io.h>
#include <simavr/sim/sim_core.h>
#include <simavr/sim/avr_ioport.h>
#include <simavr/sim/avr_uart.h>
#include <simavr/sim/sim_vcd_file.h>

#include <boost/circular_buffer.hpp>
#include <boost/function.hpp>

#include <libucomm/envelope.h>
#include <libucomm/checksum.h>

#include "drc_proto.h"
#include "../protocol/drc_proto_constants.h"

#include <catch_ros/catch.hpp>

namespace Catch
{
	inline std::string toString(uint8_t c)
	{
		std::ostringstream ss;
		ss << "0x" << std::hex << (unsigned int)c;
		return ss.str();
	}
}

typedef uc::EnvelopeWriter<uc::InvertedModSumGenerator> EnvelopeWriter;
typedef uc::IO<EnvelopeWriter, uc::IO_W> SimpleWriter;
typedef Proto<SimpleWriter> WProto;

typedef uc::EnvelopeReader<uc::InvertedModSumGenerator, 256> EnvelopeReader;
typedef uc::IO<EnvelopeReader, uc::IO_R> SimpleReader;
typedef Proto<SimpleReader> RProto;

class PCUART : public uc::CharWriter
{
public:
	PCUART();

	void attach(avr_t* avr, char name);

	EnvelopeReader* reader()
	{ return &m_reader; }

	void setMsgCallback(const boost::function<void(EnvelopeReader*)>& cb)
	{
		m_cb = cb;
	}

	virtual bool writeChar(uint8_t c) override;

	void flush()
	{}

private:
	static void uart_in_hook(struct avr_irq_t* irq, uint32_t value, void* param);
	static void uart_xon_hook(struct avr_irq_t* irq, uint32_t value, void* param);
	static void uart_xoff_hook(struct avr_irq_t* irq, uint32_t value, void* param);

	EnvelopeReader m_reader;
	boost::function<void(EnvelopeReader*)> m_cb;

	boost::circular_buffer<uint8_t> m_inBuf;

	bool m_xon;
	avr_irq_t* m_inIrq;
};

struct DXLUART
{
public:
	DXLUART(unsigned int num_servos, uint8_t ttl_base_address);

	void attach(avr_t* avr, char name);

	uint32_t dxlRegister(uint8_t id, uint16_t addr, uint8_t len) const;
	uint32_t ttlRegister(uint16_t addr, uint8_t len) const;

	void setDXLRegister(uint8_t id, uint16_t addr, uint8_t len, uint32_t value);
	void setTTLRegister(uint16_t addr, uint8_t len, uint32_t value);

	void flush()
	{}
private:
	enum ParserState
	{
		STATE_INIT,
		STATE_ID,
		STATE_LENGTH,
		STATE_INSTRUCTION,
		STATE_PARAMETERS,
		STATE_CHECKSUM
	};

	static void uart_in_hook(struct avr_irq_t* irq, uint32_t value, void* param);
	static void uart_xoff_hook(struct avr_irq_t* irq, uint32_t value, void* param);
	static void uart_xon_hook(struct avr_irq_t* irq, uint32_t value, void* param);

	const char* currentStateStr() const;
	void handlePacket();
	void writeStatusPacket(uint8_t id, const std::vector<uint8_t>& params, uint8_t error);
	void write(uint8_t c);

	ParserState m_state;
	int m_subIdx;
	unsigned int m_packetID;
	unsigned int m_packetLength;
	int m_packetInstruction;
	int m_packetChecksum;

	std::vector<uint8_t> m_stuffedParams;

	std::vector<std::vector<uint8_t>> m_servos;

	std::vector<uint8_t> m_ttlBridge;

	avr_irq_t* m_inIrq;

	uint16_t m_txChecksum;

    uint8_t m_ttl_base_address;

	bool m_xon;
	boost::circular_buffer<uint8_t> m_txBuf;
};

avr_t* initAVR();

void stepAVR(avr_t* avr, uint64_t cycles);

#endif
