/*
 * Copyright (c) 2009-2011, Fabian Greif
 * Copyright (c) 2010, Martin Rosekeit
 * Copyright (c) 2010, Thorsten Lajewski
 * Copyright (c) 2012-2015, 2017-2018, Niklas Hauser
 * Copyright (c) 2014, 2017, Sascha Schade
 * Copyright (c) 2023, Nick Fiege
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#ifndef MODM_MCP2515_DMAINT_HPP
#error "Don't include this file directly, use 'mcp2515.hpp' instead!"
#endif
#include <cstring>
#include <modm/architecture/interface/assert.hpp>

#include "mcp2515_bit_timings.hpp"
#include "mcp2515_definitions.hpp"
#include "mcp2515_options.hpp"

// ----------------------------------------------------------------------------

template<typename SPI, typename CS, typename INT>
bool
modm::Mcp2515DmaInt<SPI, CS, INT>::initializeWithPrescaler(uint8_t prescaler /* 2 .. 128 */,
													 uint8_t sjw /* in 1TQ .. 3TQ */,
													 uint8_t prop /* in 1TQ .. 8TQ */,
													 uint8_t ps1 /* in 1TQ .. 8TQ */,
													 uint8_t ps2 /* in 2TQ .. 8TQ */)
{
	// Build CNF1 .. 3 from parameters
	// Configuration is stored at increasing addresses in MCP2515,
	// so prepare CNF3, CNF2 and CNF1 in that order.
	uint8_t cnf[3] = {0};
	static constexpr uint8_t CNF1_idx = 2;
	static constexpr uint8_t CNF2_idx = 1;
	static constexpr uint8_t CNF3_idx = 0;
	cnf[CNF1_idx] = ((sjw - 1) << 6) | ((prescaler / 2 - 1) & 0x3f);
	cnf[CNF2_idx] = (1 << 7) | ((ps1 - 1) << 3) | ((prop - 1) << 0);
	cnf[CNF3_idx] = (ps2 - 1);

	using namespace mcp2515;

	// software reset for the mcp2515, after this the chip is back in the
	// configuration mode
	chipSelect.reset();
	spi.transferBlocking(RESET, configuration);
	// modm::delay_ms(1);
	chipSelect.set();

	// wait a bit to give the MCP2515 some time to restart
	modm::delay_ms(30);

	chipSelect.reset();
	spi.transferBlocking(WRITE, configuration);
	spi.transferBlocking(CNF3, configuration);

	// load CNF1..3
	spi.transferBlocking(cnf, nullptr, 3, configuration);

	// enable interrupts
	spi.transferBlocking(RX1IE | RX0IE, configuration);
	chipSelect.set();

	// set TXnRTS pins as inwrites
	writeRegister(TXRTSCTRL, 0);

	// disable RXnBF pins (high impedance state)
	writeRegister(BFPCTRL, 0);

	// check if we could read back some of the values
	uint8_t readback = readRegister(CNF2);

	if (not modm_assert_continue_fail_debug(readback == cnf[CNF2_idx], "mcp2515.init",
											"Cannot read the CNF2 register of the MCP2515!",
											readback))
		return false;

	// reset device to normal mode and disable the clkout pin and
	// wait until the new mode is active
	writeRegister(CANCTRL, 0);
	while ((readRegister(CANSTAT) & (OPMOD2 | OPMOD1 | OPMOD0)) != 0) {}

	return true;
}

template<typename SPI, typename CS, typename INT>
template<modm::frequency_t externalClockFrequency, modm::bitrate_t bitrate,
		 modm::percent_t tolerance>
bool
modm::Mcp2515DmaInt<SPI, CS, INT>::initialize()
{
	modm::platform::Exti::enableInterrupts<INT>();
	modm::platform::Exti::connect<INT>(modm::platform::Exti::Trigger::FallingEdge, [&](uint8_t /*line*/) mutable {
		using namespace mcp2515;
		__disable_irq();  // disable all interrupts
		mcp2515ReadMessage();
		__enable_irq();   // enable all interrupts
	});

	using Timings = modm::CanBitTimingMcp2515<externalClockFrequency, bitrate>;
	return initializeWithPrescaler(Timings::getPrescaler(), Timings::getSJW(), Timings::getProp(),
								   Timings::getPS1(), Timings::getPS2());
}

// ----------------------------------------------------------------------------
template<typename SPI, typename CS, typename INT>
void
modm::Mcp2515DmaInt<SPI, CS, INT>::setFilter(accessor::Flash<uint8_t> filter)
{
	using namespace mcp2515;

	// change to configuration mode
	bitModify(CANCTRL, 0xe0, REQOP2);

	while ((readRegister(CANSTAT) & 0xe0) != REQOP2)
		;
	writeRegister(RXB0CTRL, BUKT);
	writeRegister(RXB1CTRL, 0);

	uint8_t i, j;
	for (i = 0; i < 0x30; i += 0x10)
	{
		chipSelect.reset();
		spi.transferBlocking(WRITE, configuration);
		spi.transferBlocking(i, configuration);

		for (j = 0; j < 12; j++)
		{
			if (i == 0x20 && j >= 0x08) break;

			spi.transferBlocking(*filter++, configuration);
		}
		chipSelect.set();
	}
	chipSelect.set();
	bitModify(CANCTRL, 0xe0, 0);
}

// ----------------------------------------------------------------------------
template<typename SPI, typename CS, typename INT>
void
modm::Mcp2515DmaInt<SPI, CS, INT>::setMode(Can::Mode mode)
{
	using namespace mcp2515;

	uint8_t reg = 0;
	if (mode == Can::Mode::ListenOnly)
	{
		reg = REQOP1 | REQOP0;
	} else if (mode == Can::Mode::LoopBack)
	{
		reg = REQOP1;
	}

	// set the new mode
	bitModify(CANCTRL, REQOP2 | REQOP1 | REQOP0, reg);

	while ((readRegister(CANSTAT) & (OPMOD2 | OPMOD1 | OPMOD0)) != reg)
	{
		// wait for the new mode to become active
	}
}

// ----------------------------------------------------------------------------

template<typename SPI, typename CS, typename INT>
bool
modm::Mcp2515DmaInt<SPI, CS, INT>::isMessageAvailable()
{
	return rxQueue.isNotEmpty();
}

template<typename SPI, typename CS, typename INT>
bool
modm::Mcp2515DmaInt<SPI, CS, INT>::getMessage(can::Message &message, uint8_t * /*filter_id*/)
{
	using namespace mcp2515;

	if (rxQueue.isEmpty())
	{
		// no message in the receive buffer
		return false;
	} else
	{
		auto &rxMessage = rxQueue.get();
		memcpy(&message, &rxMessage, sizeof(message));
		rxQueue.pop();
		return true;
	}
}

// ----------------------------------------------------------------------------
template<typename SPI, typename CS, typename INT>
bool
modm::Mcp2515DmaInt<SPI, CS, INT>::isReadyToSend()
{
	using namespace mcp2515;

	return txQueue.isNotFull();
}

// ----------------------------------------------------------------------------
template<typename SPI, typename CS, typename INT>
bool
modm::Mcp2515DmaInt<SPI, CS, INT>::sendMessage(const can::Message &message)
{
	using namespace mcp2515;

	// if (not modm_assert_continue_ignore(txQueue.push(message), "mcp2515.can.tx",
	// 									"CAN transmit software buffer overflowed!", 1))
	// {
	// 	/// buffer full, could not send
	// 	return false;
	// }
	modm::platform::Exti::disableInterrupts<INT>();
	mcp2515SendMessage(message);
	modm::platform::Exti::enableInterrupts<INT>();
	return true;
}

// ----------------------------------------------------------------------------
template<typename SPI, typename CS, typename INT>
void
modm::Mcp2515DmaInt<SPI, CS, INT>::mcp2515ReadMessage()
{
	using namespace mcp2515;


	auto readData= [&]() {
		mcpSpiDataStruct* mcpSpiDataPtr = reinterpret_cast<mcpSpiDataStruct*>(rx_buf);

		// we now have all data inside rx_buf
		// first message
		if (mcpSpiDataPtr->msg1.IDE){			
			messageBuffer.flags.extended = true;
			messageBuffer.identifier = mcpSpiDataPtr->msg1.EID;
		}
		else{			
			messageBuffer.flags.extended = false;
			messageBuffer.identifier = mcpSpiDataPtr->msg1.SID;
		}
		messageBuffer.flags.rtr = mcpSpiDataPtr->msg1.RTR;
		messageBuffer.length = mcpSpiDataPtr->msg1.DLC;
		
		if (not modm_assert_continue_ignore(rxQueue.push(messageBuffer), "mcp2515.can.tx",
			"CAN transmit software buffer overflowed!", 1)){}

		// prepare next status read
		tx_buf[0] = RX_STATUS;
		tx_buf[1] = 0xFF;
	};

	auto processStatusAndPrepareRead = [&]() {
		statusBufferR = rx_buf[1];
		readSuccessfulFlag = true;

		if (statusBufferR & FLAG_RXB0_FULL)
		{
			addressBufferR = READ_RX ;  // message in buffer 0
			spiReadDataLength = 1+13 + 1;			
		}
		else if (statusBufferR & FLAG_RXB1_FULL)
		{
			addressBufferR = READ_RX | 0x04;  // message in buffer 1 (RXB1SIDH)
			spiReadDataLength = 1+13 + 1;			
		}
		else
		{
			readSuccessfulFlag = false;  // Error: no message available
			spiReadDataLength = 0;
		}

		if (readSuccessfulFlag)
		{
			tx_buf[0] = addressBufferR;
			tx_buf[1] = 0xff;
		}
	};

	auto processStatusAfterRead = [&]() {
		statusBufferR = rx_buf[1];
		readSuccessfulFlag = true;

		if (statusBufferR & FLAG_RXB0_FULL)
		{
			addressBufferR = READ_RX ;  // message in buffer 0
			spiReadDataLength = 1+13 + 1;
		}
		else if (statusBufferR & FLAG_RXB1_FULL)
		{
			addressBufferR = READ_RX | 0x04;  // message in buffer 1 (RXB1SIDH)
			spiReadDataLength = 1+13 + 1;
		}
		 else
		{
			readSuccessfulFlag = false;  // Error: no message available
			spiReadDataLength = 0;
		}

		if (readSuccessfulFlag){
			mcp2515ReadMessage();
		}

	};
	////////////////////////////////////////////////////////////////////////////

	// MCP2515 Registers
	// 0x61 / 0x71 : RXBnSIDH : RECEIVE BUFFER n STANDARD IDENTIFIER REGISTER HIGH
	// 0x62 / 0x72 : RXBnSIDL : RECEIVE BUFFER n STANDARD IDENTIFIER REGISTER LOW
	// 0x63 / 0x73 : RXBnEID8 : RECEIVE BUFFER n EXTENDED IDENTIFIER REGISTER HIGH
	// 0x64 / 0x74 : RXBnEID0: RECEIVE BUFFER n EXTENDED IDENTIFIER REGISTER LOW

	// 0x65 / 0x75 : RXBnDLC: RECEIVE BUFFER n DATA LENGTH CODE REGISTER

	// 0x66 - 0x6D / 0x76 - 0x7D : RXBnDm: RECEIVE BUFFER n DATA BYTE m REGISTER


	tx_buf[0] = RX_STATUS;
	tx_buf[1] = 0xFF;
	spi.pipeline(
		SpiTransferStep{tx_buf, rx_buf, 2, processStatusAndPrepareRead, nullptr, configuration, CsBehavior<CS>(ChipSelect::TOGGLE)},
		SpiTransferStep{tx_buf, rx_buf, [&](){return spiReadDataLength;}, readData, [&](){return readSuccessfulFlag;}, configuration, CsBehavior<CS>(ChipSelect::TOGGLE)},
		SpiTransferStep{tx_buf, rx_buf, 2, processStatusAfterRead, nullptr, configuration, CsBehavior<CS>(ChipSelect::TOGGLE)}
	);
}

/*
template<typename SPI, typename CS, typename INT>
void
modm::Mcp2515DmaInt<SPI, CS, INT>::mcp2515ReadMessage()
{
	using namespace mcp2515;

	auto copyResultCanMessage = [&]() {
		std::memcpy(messageBuffer.data, rx_buf, messageBuffer.length);
		if (not modm_assert_continue_ignore(rxQueue.push(messageBuffer), "mcp2515.can.tx",
			"CAN transmit software buffer overflowed!", 1)){}
	};
	auto readCanMessage = [&]() {
		const uint32_t *ptr = &messageBuffer.identifier;
		messageBuffer.flags.extended = false;

		if (rx_buf[2] & MCP2515_IDE)
		{
			*((uint16_t*)ptr + 1) = (uint16_t)rx_buf[1] << 5;
			*((uint8_t*)ptr + 1) = rx_buf[3];
			*((uint8_t*)ptr + 2) |= (rx_buf[2] >> 3) & 0x1C;
			*((uint8_t*)ptr + 2) |= rx_buf[2] & 0x03;
			*((uint8_t*)ptr) = rx_buf[4];
			messageBuffer.flags.extended = true;
		} else
		{
			*((uint8_t*)ptr + 3) = 0;
			*((uint8_t*)ptr + 2) = 0;
			*((uint16_t*)ptr) = (uint16_t)rx_buf[1] << 3;
			*((uint8_t*)ptr) |= rx_buf[2] >> 5;
		}
		if (statusBufferR & FLAG_RTR)
		{
			messageBuffer.flags.rtr = true;
		} else
		{
			messageBuffer.flags.rtr = false;
		}
		messageBuffer.length = rx_buf[5] & 0x0f;

		// new DMA transfer, because we know buffer length now
		std::memset(tx_buf, 0xFF, messageBuffer.length);
	};
	auto processStatusAndPrepareRead = [&]() {
		statusBufferR = rx_buf[1];
		readSuccessfulFlag = true;

		if (statusBufferR & (FLAG_RXB0_FULL | FLAG_RXB1_FULL))
		{
			// data in both registers, begin read with register 0:
			addressBufferR = READ_RX;  // message in buffer 0
			spiReadDataLength = 12;

		} else if (statusBufferR & FLAG_RXB0_FULL)
		{
			addressBufferR = READ_RX ;  // message in buffer 0
			spiReadDataLength = 6;
		}
		else if (statusBufferR & FLAG_RXB1_FULL)
		{
			addressBufferR = READ_RX | 0x04;  // message in buffer 1 (RXB1SIDH)
			spiReadDataLength = 6;
		}
		 else
		{
			readSuccessfulFlag = false;  // Error: no message available
			spiReadDataLength = 0;
		}

		if (readSuccessfulFlag)
		{
			tx_buf[0] = addressBufferR;
			tx_buf[1] = 0xff;
			tx_buf[2] = 0xff;
			tx_buf[3] = 0xff;
			tx_buf[4] = 0xff;
			tx_buf[5] = 0xff;
		}
	};
	////////////////////////////////////////////////////////////////////////////

	// MCP2515 Registers
	// 0x61 / 0x71 : RXBnSIDH : RECEIVE BUFFER n STANDARD IDENTIFIER REGISTER HIGH
	// 0x62 / 0x72 : RXBnSIDL : RECEIVE BUFFER n STANDARD IDENTIFIER REGISTER LOW
	// 0x63 / 0x73 : RXBnEID8 : RECEIVE BUFFER n EXTENDED IDENTIFIER REGISTER HIGH
	// 0x64 / 0x74 : RXBnEID0: RECEIVE BUFFER n EXTENDED IDENTIFIER REGISTER LOW

	// 0x65 / 0x75 : RXBnDLC: RECEIVE BUFFER n DATA LENGTH CODE REGISTER

	// 0x66 - 0x6D / 0x76 - 0x7D : RXBnDm: RECEIVE BUFFER n DATA BYTE m REGISTER


	tx_buf[0] = RX_STATUS;
	tx_buf[1] = 0xFF;
	spi.pipeline(
		SpiTransferStep{tx_buf, rx_buf, 2, processStatusAndPrepareRead, nullptr, configuration, CsBehavior<CS>(ChipSelect::TOGGLE)},
		SpiTransferStep{tx_buf, rx_buf, [&](){return spiReadDataLength;}, readCanMessage, [&](){return readSuccessfulFlag;}, configuration, CsBehavior<CS>(ChipSelect::RESET)},
		SpiTransferStep{tx_buf, rx_buf, [&](){return messageBuffer.length;}, copyResultCanMessage, [&](){return readSuccessfulFlag;}, configuration, CsBehavior<CS>(ChipSelect::SET)}
	);
}
*/
// ----------------------------------------------------------------------------
template<typename SPI, typename CS, typename INT>
bool
modm::Mcp2515DmaInt<SPI, CS, INT>::mcp2515IsReadyToSend(uint8_t status)
{
	using namespace mcp2515;
	if ((status & (TXB2CNTRL_TXREQ | TXB1CNTRL_TXREQ | TXB0CNTRL_TXREQ)) ==
		(TXB2CNTRL_TXREQ | TXB1CNTRL_TXREQ | TXB0CNTRL_TXREQ))
	{
		// all buffers currently in use
		return false;
	}
	return true;
}

// ----------------------------------------------------------------------------
template<typename SPI, typename CS, typename INT>
void
modm::Mcp2515DmaInt<SPI, CS, INT>::mcp2515SendMessage(const can::Message &message)
{
	using namespace modm::mcp2515;

	/// put can message ionto our tx message buffer
	auto statusPost = [message](){
		statusBufferS = rx_buf[1];
		addressBufferS = static_cast<uint8_t>(false);
		if(mcp2515IsReadyToSend(statusBufferS)){
			if ((statusBufferS & TXB0CNTRL_TXREQ) == 0)
			{
				addressBufferS = 0x00;  // TXB0SIDH
			} else if ((statusBufferS & TXB1CNTRL_TXREQ) == 0)
			{
				addressBufferS = 0x02;  // TXB1SIDH
			} else if ((statusBufferS & TXB2CNTRL_TXREQ) == 0)
			{
				addressBufferS = 0x04;  // TXB2SIDH
			} else
			{
				// all buffer are in use => could not send the message
				addressBufferS = 0xFF;  // invalid
			}

			if (addressBufferS == 0x00 || addressBufferS == 0x02 || addressBufferS == 0x04){
				
				// RF_CALL(spi.transfer(WRITE_TX | addressBufferS));
				tx_buf[0] = WRITE_TX | addressBufferS;

				// fill in 4 bytes:
				writeIdentifierBuffer(message.identifier, message.flags.extended, &tx_buf[0], 1);

				// if the message is a rtr-frame, is has a length but no attached data
				if (message.flags.rtr)
				{
					// RF_CALL(spi.transfer(MCP2515_RTR | message.length));
					tx_buf[5] = MCP2515_RTR | message.length;
				} else
				{
					// RF_CALL(spi.transfer(message.length));
					tx_buf[5] = message.length;
					// payload:
					std::memcpy(&tx_buf[6], message.data, message.length);
				}
			}
		}
	};

	auto identifierPost = [&](){
		addressBufferS = (addressBufferS == 0) ? 1 : addressBufferS;  // 0 2 4 => 1 2 4
		tx_buf[0] = RTS | addressBufferS;
	};

	// go
	tx_buf[0] = READ_STATUS;
	tx_buf[1] = 0xFF;
	spi.pipeline(
		SpiTransferStep{tx_buf, rx_buf, 2, statusPost, nullptr, configuration, CsBehavior<CS>(ChipSelect::TOGGLE)},
		SpiTransferStep{tx_buf, rx_buf, [message](){return 6 + message.length;}, identifierPost, [&](){return addressBufferS != 0xff;}, configuration, CsBehavior<CS>(ChipSelect::TOGGLE)},
		SpiTransferStep{tx_buf, rx_buf, 1, nullptr, [&](){return addressBufferS != 0xFF;}, configuration, CsBehavior<CS>(ChipSelect::TOGGLE)} // skip if no free tx buffer
	);
}
// ----------------------------------------------------------------------------

template<typename SPI, typename CS, typename INT>
void
modm::Mcp2515DmaInt<SPI, CS, INT>::writeRegister(uint8_t address, uint8_t data)
{
	using namespace mcp2515;

	chipSelect.reset();
	spi.transferBlocking(WRITE, configuration);
	spi.transferBlocking(address, configuration);
	spi.transferBlocking(data, configuration);
	chipSelect.set();
}

template<typename SPI, typename CS, typename INT>
uint8_t
modm::Mcp2515DmaInt<SPI, CS, INT>::readRegister(uint8_t address)
{
	using namespace mcp2515;

	chipSelect.reset();
	spi.transferBlocking(READ, configuration);
	spi.transferBlocking(address, configuration);
	uint8_t data = spi.transferBlocking(0xff, configuration);
	chipSelect.set();
	return data;
}

template<typename SPI, typename CS, typename INT>
void
modm::Mcp2515DmaInt<SPI, CS, INT>::bitModify(uint8_t address, uint8_t mask, uint8_t data)
{
	using namespace mcp2515;

	chipSelect.reset();
	spi.transferBlocking(BIT_MODIFY, configuration);
	spi.transferBlocking(address, configuration);
	spi.transferBlocking(mask, configuration);
	spi.transferBlocking(data, configuration);
	chipSelect.set();
}

// ----------------------------------------------------------------------------

template<typename SPI, typename CS, typename INT>
void
modm::Mcp2515DmaInt<SPI, CS, INT>::writeIdentifierBuffer(const uint32_t &identifier, bool isExtendedFrame,
												   uint8_t *buffer, uint8_t start_index)
{
	using namespace mcp2515;
	const uint32_t *ptr = &identifier;

	// RF_WAIT_UNTIL(this->acquireMaster());
	if (isExtendedFrame)
	{

		// RF_CALL(spi.transfer(*((uint16_t *)ptr + 1) >> 5));
		buffer[start_index] = *((uint16_t *)ptr + 1) >> 5;

		// calculate the next values
		a = (*((uint8_t *)ptr + 2) << 3) & 0xe0;
		a |= MCP2515_IDE;
		a |= (*((uint8_t *)ptr + 2)) & 0x03;
		// RF_CALL(spi.transfer(a));
		buffer[start_index + 1] = a;

		// RF_CALL(spi.transfer(*((uint8_t *)ptr + 1)));
		buffer[start_index + 2] = *((uint8_t *)ptr + 1);
		// RF_CALL(spi.transfer(*((uint8_t *)ptr)));
		buffer[start_index + 3] = *((uint8_t *)ptr);
	} else
	{
		buffer[start_index] = *((uint16_t *)ptr) >> 3;
		buffer[start_index + 1] = *((uint8_t *)ptr) << 5;
		buffer[start_index + 2] = 0;
		buffer[start_index + 3] = 0;
	}
}
