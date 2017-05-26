// Support routines for unit testing the firmware
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "test_support.h"

#define LOG_COMM 0

static uint16_t update_crc(uint16_t crc_accum, uint8_t data)
{
	uint16_t i;

	static const uint16_t crc_table[256] = {
		0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
		0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
		0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
		0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
		0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
		0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
		0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
		0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
		0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
		0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
		0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
		0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
		0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
		0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
		0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
		0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
		0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
		0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
		0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
		0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
		0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
		0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
		0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
		0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
		0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
		0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
		0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
		0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
		0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
		0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
		0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
		0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
	};

	i = ((uint16_t)(crc_accum >> 8) ^ data) & 0xFF;
	crc_accum = (crc_accum << 8) ^ crc_table[i];

	return crc_accum;
}

PCUART::PCUART()
 : m_inBuf(1024)
{}

void PCUART::attach(avr_t* avr, char name)
{
	m_xon = true;

	avr_irq_t * src =  avr_io_getirq(avr, AVR_IOCTL_UART_GETIRQ(name), UART_IRQ_OUTPUT);
	avr_irq_t * dst =  avr_io_getirq(avr, AVR_IOCTL_UART_GETIRQ(name), UART_IRQ_INPUT);
	avr_irq_t * xon =  avr_io_getirq(avr, AVR_IOCTL_UART_GETIRQ(name), UART_IRQ_OUT_XON);
	avr_irq_t * xoff = avr_io_getirq(avr, AVR_IOCTL_UART_GETIRQ(name), UART_IRQ_OUT_XOFF);

	REQUIRE(src);
	REQUIRE(dst);
	REQUIRE(xon);
	REQUIRE(xoff);

	m_inIrq = dst;
	avr_irq_register_notify(src, &PCUART::uart_in_hook, this);
	avr_irq_register_notify(xon, &PCUART::uart_xon_hook, this);
	avr_irq_register_notify(xoff, &PCUART::uart_xoff_hook, this);

	// Disable automatic stdout of simavr
	uint32_t f = 0;
	avr_ioctl(avr, AVR_IOCTL_UART_GET_FLAGS(name), &f);
	f &= ~AVR_UART_FLAG_STDIO;
	avr_ioctl(avr, AVR_IOCTL_UART_SET_FLAGS(name), &f);
}

bool PCUART::writeChar(uint8_t c)
{
	if(m_xon)
	{
		REQUIRE(m_inBuf.empty());
#if LOG_COMM
		printf("PC -> µC    DXL: 0x%02x\n", c);
#endif
		avr_raise_irq(m_inIrq, c);
	}
	else
		m_inBuf.push_back(c);

	return true;
}

void PCUART::uart_in_hook(struct avr_irq_t* irq, uint32_t value, void* param)
{
#if LOG_COMM
	printf("PC <- µC    DXL: 0x%02x\n", value);
#endif
	PCUART* uart = reinterpret_cast<PCUART*>(param);
	EnvelopeReader::TakeResult res = uart->m_reader.take(value);

	REQUIRE(res != EnvelopeReader::CHECKSUM_ERROR);

	if(res == EnvelopeReader::NEW_MESSAGE)
	{
		if(uart->m_cb)
			uart->m_cb(&uart->m_reader);
	}
}

void PCUART::uart_xon_hook(struct avr_irq_t* irq, uint32_t value, void* param)
{
	PCUART* uart = reinterpret_cast<PCUART*>(param);

	uart->m_xon = true;

	while(uart->m_xon && !uart->m_inBuf.empty())
	{
		uint8_t c = uart->m_inBuf.front();
		uart->m_inBuf.pop_front();

#if LOG_COMM
		printf("PC -> µC    DXL: 0x%02x\n", c);
#endif
		avr_raise_irq(uart->m_inIrq, c);
	}
}

void PCUART::uart_xoff_hook(struct avr_irq_t* irq, uint32_t value, void* param)
{
	PCUART* uart = reinterpret_cast<PCUART*>(param);
	uart->m_xon = false;
}

void DXLUART::attach(avr_t* avr, char name)
{
	avr_irq_t * src =  avr_io_getirq(avr, AVR_IOCTL_UART_GETIRQ(name), UART_IRQ_OUTPUT);
	avr_irq_t * dst =  avr_io_getirq(avr, AVR_IOCTL_UART_GETIRQ(name), UART_IRQ_INPUT);
	avr_irq_t * xon =  avr_io_getirq(avr, AVR_IOCTL_UART_GETIRQ(name), UART_IRQ_OUT_XON);
	avr_irq_t * xoff = avr_io_getirq(avr, AVR_IOCTL_UART_GETIRQ(name), UART_IRQ_OUT_XOFF);

	REQUIRE(src);
	REQUIRE(dst);
	REQUIRE(xon);
	REQUIRE(xoff);

	m_xon = true;

	avr_irq_register_notify(src, &DXLUART::uart_in_hook, this);
	avr_irq_register_notify(xoff, &DXLUART::uart_xoff_hook, this);
	avr_irq_register_notify(xon, &DXLUART::uart_xon_hook, this);
	m_inIrq = dst;

	// Disable automatic stdout of simavr
	uint32_t f = 0;
	avr_ioctl(avr, AVR_IOCTL_UART_GET_FLAGS(name), &f);
	f &= ~AVR_UART_FLAG_STDIO;
	avr_ioctl(avr, AVR_IOCTL_UART_SET_FLAGS(name), &f);
}

DXLUART::DXLUART(unsigned int numServos, uint8_t ttl_base_address)
 : m_state(STATE_INIT)
 , m_subIdx(0)
 , m_ttl_base_address(ttl_base_address)
 , m_txBuf(1024)
{
	m_servos.resize(numServos);

	for(unsigned int i = 0; i < numServos; ++i)
	{
		m_servos[i].resize(1024);

		m_servos[i][7] = i+1; // ID
	}

	m_ttlBridge.resize(1024);
}

const char* DXLUART::currentStateStr() const
{
	switch(m_state)
	{
		case STATE_INIT:              return "INIT";
		case STATE_ID:                return "ID";
		case STATE_LENGTH:            return "LENGTH";
		case STATE_INSTRUCTION:       return "INSTRUCTION";
		case STATE_PARAMETERS:        return "PARAMETERS";
		case STATE_CHECKSUM:          return "CHECKSUM";
		default:                      return "UNKNOWN";
	}
}

const uint8_t DXL_HEADER[] = {0xFF, 0xFF, 0xFD, 0x00};

void DXLUART::uart_in_hook(struct avr_irq_t* irq, uint32_t value, void* param)
{
	DXLUART* uart = reinterpret_cast<DXLUART*>(param);
#if LOG_COMM
	printf("PC    µC -> DXL: 0x%02x (%s)\n", value, uart->currentStateStr());
#endif

	switch(uart->m_state)
	{
		case STATE_INIT:
			REQUIRE(value == DXL_HEADER[uart->m_subIdx]);
			if(++uart->m_subIdx == sizeof(DXL_HEADER))
				uart->m_state = STATE_ID;
			break;
		case STATE_ID:
			uart->m_packetID = value;
			REQUIRE(value >= 1);
			REQUIRE(value < 255);
			uart->m_state = STATE_LENGTH;
			uart->m_subIdx = 0;
			uart->m_packetLength = 0;
			break;
		case STATE_LENGTH:
			if(uart->m_subIdx == 0)
			{
				uart->m_packetLength = value;
				uart->m_subIdx = 1;
			}
			else
			{
				uart->m_packetLength |= (value << 8);
				uart->m_state = STATE_INSTRUCTION;
			}
			break;
		case STATE_INSTRUCTION:
			uart->m_packetInstruction = value;
			if(uart->m_packetLength > 3)
			{
				uart->m_state = STATE_PARAMETERS;
				uart->m_stuffedParams.clear();
			}
			else
			{
				uart->m_subIdx = 0;
				uart->m_state = STATE_CHECKSUM;
			}
			break;
		case STATE_PARAMETERS:
			uart->m_stuffedParams.push_back(value);
			if(uart->m_stuffedParams.size() == uart->m_packetLength-3)
			{
				uart->m_subIdx = 0;
				uart->m_state = STATE_CHECKSUM;
			}
			break;
		case STATE_CHECKSUM:
			if(uart->m_subIdx == 0)
			{
				uart->m_packetChecksum = value;
				uart->m_subIdx = 1;
			}
			else
			{
				uart->m_packetChecksum |= (value << 8);

				uint16_t crc = 0;
				for(uint8_t c : DXL_HEADER)
					crc = update_crc(crc, c);
				crc = update_crc(crc, uart->m_packetID);
				crc = update_crc(crc, uart->m_packetLength & 0xFF);
				crc = update_crc(crc, uart->m_packetLength >> 8);
				crc = update_crc(crc, uart->m_packetInstruction);

				for(uint8_t c : uart->m_stuffedParams)
					crc = update_crc(crc, c);

				REQUIRE(crc == uart->m_packetChecksum);

				uart->handlePacket();

				uart->m_subIdx = 0;
				uart->m_state = STATE_INIT;
			}
			break;
	}
}

void DXLUART::handlePacket()
{
	// Unstuffing
	std::vector<uint8_t> params;
	params.reserve(m_stuffedParams.size());

	for(unsigned int i = 0; i < m_stuffedParams.size(); ++i)
	{
		if(i+3 < m_stuffedParams.size()
			&& m_stuffedParams[i+0] == 0xFF
			&& m_stuffedParams[i+1] == 0xFF
			&& m_stuffedParams[i+2] == 0xFD)
		{
			REQUIRE(m_stuffedParams[i+3] == 0xFD);
			params.push_back(0xFF);
			params.push_back(0xFF);
			params.push_back(0xFD);

			i += 3;
		}
		else
			params.push_back(m_stuffedParams[i]);
	}

	switch(m_packetInstruction)
	{
		case 0x83: // SYNC WRITE
		{
			REQUIRE(params.size() >= 4);

			uint16_t reg_addr = (((uint16_t)params[1]) << 8) | params[0];
			uint16_t reg_len = (((uint16_t)params[3]) << 8) | params[2];

			REQUIRE(((params.size() - 4) % (reg_len + 1)) == 0);
			REQUIRE((reg_addr + reg_len) < 1024);
#if LOG_COMM
			printf("DXL: Got SyncWrite packet, register addr %d (0x%X), length %d\n", reg_addr, reg_addr, reg_len);
#endif
			for(unsigned int i = 4; i < params.size(); i += reg_len + 1)
			{
				uint8_t id = params[i];

				if(id == m_ttl_base_address)
				{
					for(unsigned int j = 0; j < reg_len; ++j)
						m_ttlBridge[reg_addr + j] = params[i + 1 + j];
				}
				else
				{
					REQUIRE(id > 0);
					REQUIRE(id <= m_servos.size());

					for(unsigned int j = 0; j < reg_len; ++j)
						m_servos[id-1][reg_addr + j] = params[i + 1 + j];
				}
			}

			break;
		}
		case 0x92: // BULK READ
		{
			REQUIRE(params.size() > 0);

			unsigned int index = 0;
#if LOG_COMM
			printf("DXL: Got BulkRead packet\n");
#endif
			while(index < params.size())
			{
				REQUIRE(params.size() >= index + 5);

				uint8_t id = params[index];
				uint16_t reg_addr = (((uint16_t)params[index + 2]) << 8) | params[index + 1];
				uint16_t reg_len = (((uint16_t)params[index + 4]) << 8) | params[index + 3];

				REQUIRE(id > 0);
				REQUIRE((reg_addr + reg_len) <= 1024);
#if LOG_COMM
				printf(" ^- Bulkread: writing answer for %2d\n", id);
#endif
				// Write reply packet
				std::vector<uint8_t> params;
				for(uint16_t addr = reg_addr; addr < reg_addr + reg_len; ++addr)
					params.push_back(m_servos[id-1][addr]);

				writeStatusPacket(id, params, 0);

				index += 5;
			}
#if LOG_COMM
			printf("\n");
#endif
			break;
		}
		default:
			FAIL("Got unknown packet with instruction " << m_packetInstruction);
			break;
	}
}

void DXLUART::writeStatusPacket(uint8_t id, const std::vector<uint8_t>& params, uint8_t error)
{
	m_txChecksum = 0;

	std::vector<uint8_t> stuffed;
	stuffed.reserve(2*params.size());

	for(unsigned int i = 0; i < params.size(); ++i)
	{
		if(i + 3 <= params.size()
			&& params[i] == 0xFF
			&& params[i+1] == 0xFF
			&& params[i+2] == 0xFD)
		{
			stuffed.push_back(0xFF);
			stuffed.push_back(0xFF);
			stuffed.push_back(0xFD);
			stuffed.push_back(0xFD);
			i += 2;
		}
		else
			stuffed.push_back(params[i]);
	}

	write(0xFF);
	write(0xFF);
	write(0xFD);
	write(0x00);

	// ID
	write(id);

	// Length
	write((stuffed.size() + 4) & 0xFF);
	write((stuffed.size() + 4) >> 8);

	// Instruction
	write(0x55);

	// Error
	write(error);

	// Parameters
	for(uint8_t c : stuffed)
		write(c);

	// Checksum
	uint16_t checksum = m_txChecksum;
	write(checksum & 0xFF);
	write(checksum >> 8);
}

void DXLUART::write(uint8_t c)
{
	m_txChecksum = update_crc(m_txChecksum, c);

	if(m_xon)
	{
		REQUIRE(m_txBuf.empty());
		avr_raise_irq(m_inIrq, c);
#if LOG_COMM
		printf("PC    µC <- DXL: 0x%02x\n", c);
#endif
	}
	else
		m_txBuf.push_back(c);
}

void DXLUART::uart_xon_hook(struct avr_irq_t* irq, uint32_t value, void* param)
{
	DXLUART* uart = reinterpret_cast<DXLUART*>(param);

	uart->m_xon = true;

	while(uart->m_xon && !uart->m_txBuf.empty())
	{
		uint8_t c = uart->m_txBuf.front();
		uart->m_txBuf.pop_front();

#if LOG_COMM
		printf("PC    µC <- DXL: 0x%02x (xon)\n", c);
#endif
		avr_raise_irq(uart->m_inIrq, c);
	}
}

void DXLUART::uart_xoff_hook(struct avr_irq_t* irq, uint32_t value, void* param)
{
	DXLUART* uart = reinterpret_cast<DXLUART*>(param);
	uart->m_xon = false;
}

uint32_t DXLUART::dxlRegister(uint8_t id, uint16_t addr, uint8_t len) const
{
	REQUIRE(id > 0);
	REQUIRE(id <= m_servos.size());

	REQUIRE((addr + len) <= 1024);

	switch(len)
	{
		case 1:
			return m_servos[id-1][addr];
		case 2:
			return m_servos[id-1][addr] | (((uint16_t)m_servos[id-1][addr+1]) << 8);
		case 4:
			return m_servos[id-1][addr]
				| (((uint32_t)m_servos[id-1][addr+1]) << 8)
				| (((uint32_t)m_servos[id-1][addr+2]) << 16)
				| (((uint32_t)m_servos[id-1][addr+3]) << 24);
		default:
			FAIL("Unknown register width");
			return 0;
	}
}

void DXLUART::setDXLRegister(uint8_t id, uint16_t addr, uint8_t len, uint32_t value)
{
	REQUIRE(id > 0);
	REQUIRE(id <= m_servos.size());

	REQUIRE((addr + len) <= 1024);

	REQUIRE(len <= 4);

	while(len != 0)
	{
		m_servos[id-1][addr] = value & 0xFF;
		value >>= 8;
		len--;
		addr++;
	}
}

uint32_t DXLUART::ttlRegister(uint16_t addr, uint8_t len) const
{
	REQUIRE((addr + len) <= 1024);

	switch(len)
	{
		case 1:
			return m_ttlBridge[addr];
		case 2:
			return m_ttlBridge[addr] | (((uint16_t)m_ttlBridge[addr+1]) << 8);
		case 4:
			return m_ttlBridge[addr]
				| (((uint32_t)m_ttlBridge[addr+1]) << 8)
				| (((uint32_t)m_ttlBridge[addr+2]) << 16)
				| (((uint32_t)m_ttlBridge[addr+3]) << 24);
		default:
			FAIL("Unknown register width");
			return 0;
	}
}

void DXLUART::setTTLRegister(uint16_t addr, uint8_t len, uint32_t value)
{
	REQUIRE((addr + len) <= 1024);

	REQUIRE(len <= 4);

	while(len != 0)
	{
		m_ttlBridge[addr] = value & 0xFF;
		value >>= 8;
		len--;
		addr++;
	}
}

void pin_changed_hook(struct avr_irq_t * irq, uint32_t value, void * param)
{
	printf("DBG: %02X\n", value);
}

avr_t* initAVR()
{
	avr_t* avr = avr_make_mcu_by_name("atmega2560");
	REQUIRE(avr);

	avr_init(avr);
	avr->frequency = 16000000ULL;
	avr->log = LOG_WARNING;

	elf_firmware_t elf;
	REQUIRE(elf_read_firmware(FIRMWARE_BINARY_PATH, &elf) == 0);

	avr_load_firmware(avr, &elf);

	avr_reset(avr);

	avr->gdb_port = 1234;
	avr_gdb_init(avr);

	// Disable emergency stop
	avr_ioport_external_t e;
	e.name = 'G';
	e.mask = (1 << 1);
	e.value = 0;

	avr_ioctl(avr, AVR_IOCTL_IOPORT_SET_EXTERNAL('G'), &e);

#if LOG_COMM
	avr_irq_register_notify(avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ('F'), 8),
			pin_changed_hook,
			NULL);
#endif

	return avr;
}

void stepAVR(avr_t* avr, uint64_t cycles)
{
	int last_state = -1;

	for(uint64_t i = 0; i < cycles;)
	{
		int state = avr_run(avr);
		if(state != cpu_Running)
		{
			if(state != last_state)
				fprintf(stderr, "Strange AVR state %d, debugging?\n", state);
		}
		else
			++i;

		last_state = state;
	}
}
