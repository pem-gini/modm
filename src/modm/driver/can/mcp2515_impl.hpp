/*
 * Copyright (c) 2009-2011, Fabian Greif
 * Copyright (c) 2010, Martin Rosekeit
 * Copyright (c) 2010, Thorsten Lajewski
 * Copyright (c) 2012-2015, 2017-2018, Niklas Hauser
 * Copyright (c) 2014, 2017, Sascha Schade
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#ifndef MODM_MCP2515_HPP
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
modm::Mcp2515<SPI, CS, INT>::initializeWithPrescaler(uint8_t prescaler /* 2 .. 128 */,
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
	spi.transferBlocking(RESET);
	// modm::delay_ms(1);
	chipSelect.set();

	// wait a bit to give the MCP2515 some time to restart
	modm::delay_ms(30);

	chipSelect.reset();
	spi.transferBlocking(WRITE);
	spi.transferBlocking(CNF3);

	// load CNF1..3
	spi.transferBlocking(cnf, nullptr, 3);

	// enable interrupts
	spi.transferBlocking(RX1IE | RX0IE);
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
modm::Mcp2515<SPI, CS, INT>::initialize()
{
	using Timings = modm::CanBitTimingMcp2515<externalClockFrequency, bitrate>;

	return initializeWithPrescaler(Timings::getPrescaler(), Timings::getSJW(), Timings::getProp(),
								   Timings::getPS1(), Timings::getPS2());
}

// ----------------------------------------------------------------------------
template<typename SPI, typename CS, typename INT>
void
modm::Mcp2515<SPI, CS, INT>::setFilter(accessor::Flash<uint8_t> filter)
{
	using namespace mcp2515;

	while (!this->acquireMaster()) {};

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
		spi.transferBlocking(WRITE);
		spi.transferBlocking(i);

		for (j = 0; j < 12; j++)
		{
			if (i == 0x20 && j >= 0x08) break;

			spi.transferBlocking(*filter++);
		}
		chipSelect.set();
	}
	chipSelect.set();
	bitModify(CANCTRL, 0xe0, 0);

	while (!this->releaseMaster())
		;
}

// ----------------------------------------------------------------------------
template<typename SPI, typename CS, typename INT>
void
modm::Mcp2515<SPI, CS, INT>::setMode(Can::Mode mode)
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
modm::Mcp2515<SPI, CS, INT>::isMessageAvailable()
{
	return rxQueue.isNotEmpty();
}

template<typename SPI, typename CS, typename INT>
bool
modm::Mcp2515<SPI, CS, INT>::getMessage(can::Message &message, uint8_t * /*filter_id*/)
{
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
modm::Mcp2515<SPI, CS, INT>::isReadyToSend()
{
	return txQueue.isNotFull();
}

// ----------------------------------------------------------------------------
template<typename SPI, typename CS, typename INT>
bool
modm::Mcp2515<SPI, CS, INT>::sendMessage(const can::Message &message)
{
	if (not modm_assert_continue_ignore(txQueue.push(message), "mcp2515.can.tx",
										"CAN transmit software buffer overflowed!", 1))
	{
		/// buffer full, could not send
		return false;
	}
	return true;
}

// ----------------------------------------------------------------------------
template<typename SPI, typename CS, typename INT>
modm::ResumableResult<bool>
modm::Mcp2515<SPI, CS, INT>::mcp2515ReadMessage()
{
	using namespace mcp2515;
	const uint32_t *ptr = &messageBuffer.identifier;

	RF_BEGIN();

	// read status flag of the device
	statusBufferR = RF_CALL(readStatus(RX_STATUS));

	readSuccessfulFlag = true;
	if (statusBufferR & FLAG_RXB0_FULL)
	{
		addressBufferR = READ_RX;  // message in buffer 0
	} else if (statusBufferR & FLAG_RXB1_FULL)
	{
		addressBufferR = READ_RX | 0x04;  // message in buffer 1 (RXB1SIDH)
	} else
	{
		readSuccessfulFlag = false;  // Error: no message available
	}

	if (readSuccessfulFlag)
	{
		RF_WAIT_UNTIL(this->acquireMaster());
		chipSelect.reset();
		// RF_CALL(spi.transfer(addressBufferR));
		tx_buf[0] = addressBufferR;
		tx_buf[1] = 0xff;
		tx_buf[2] = 0xff;
		tx_buf[3] = 0xff;
		tx_buf[4] = 0xff;
		tx_buf[5] = 0xff;
		RF_CALL(spi.transfer(tx_buf, rx_buf, 6));

		messageBuffer.flags.extended = false;

		if (rx_buf[2] & MCP2515_IDE)
		{
			*((uint16_t *)ptr + 1) = (uint16_t)rx_buf[1] << 5;
			*((uint8_t *)ptr + 1) = rx_buf[3];

			*((uint8_t *)ptr + 2) |= (rx_buf[2] >> 3) & 0x1C;
			*((uint8_t *)ptr + 2) |= rx_buf[2] & 0x03;

			*((uint8_t *)ptr) = rx_buf[4];

			messageBuffer.flags.extended = true;
		} else
		{

			*((uint8_t *)ptr + 3) = 0;
			*((uint8_t *)ptr + 2) = 0;

			*((uint16_t *)ptr) = (uint16_t)rx_buf[1] << 3;

			*((uint8_t *)ptr) |= rx_buf[2] >> 5;
		}
		// RF_WAIT_UNTIL(this->releaseMaster());

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
		RF_CALL(spi.transfer(tx_buf, rx_buf, messageBuffer.length));

		std::memcpy(messageBuffer.data, rx_buf, messageBuffer.length);

		chipSelect.set();
		RF_WAIT_UNTIL(this->releaseMaster());
	}
	// RX0IF or RX1IF respectivly were already cleared automatically by rising CS.
	// See section 12.4 in datasheet.

	RF_END_RETURN(readSuccessfulFlag);
}

template<typename SPI, typename CS, typename INT>
modm::ResumableResult<void>
modm::Mcp2515<SPI, CS, INT>::update()
{
	using namespace mcp2515;

	RF_BEGIN();

	// check if the device has received a message(pin = LOW)
	// if yes: read it and put it into the rxQueue
	if (!interruptPin.read())
	{
		if (RF_CALL(mcp2515ReadMessage()))
		{
			if (not modm_assert_continue_ignore(rxQueue.push(messageBuffer), "mcp2515.can.tx",
												"CAN transmit software buffer overflowed!", 1))
			{}
		}
	}

	/// check if device accepts messages and start emptying the transmit queue if not empty
	if (txQueue.isNotEmpty())
	{
		if (RF_CALL(mcp2515SendMessage(txQueue.get()))) { txQueue.pop(); }
	}
	RF_END();
}

// ----------------------------------------------------------------------------

template<typename SPI, typename CS, typename INT>
modm::ResumableResult<bool>
modm::Mcp2515<SPI, CS, INT>::mcp2515IsReadyToSend()
{
	using namespace mcp2515;

	RF_BEGIN();

	isReadyToSendFlag = true;
	statusBufferReady = RF_CALL(readStatus(READ_STATUS));
	if ((statusBufferReady & (TXB2CNTRL_TXREQ | TXB1CNTRL_TXREQ | TXB0CNTRL_TXREQ)) ==
		(TXB2CNTRL_TXREQ | TXB1CNTRL_TXREQ | TXB0CNTRL_TXREQ))
	{
		// all buffers currently in use
		isReadyToSendFlag = false;
	}

	RF_END_RETURN(isReadyToSendFlag);
}
template<typename SPI, typename CS, typename INT>
bool
modm::Mcp2515<SPI, CS, INT>::mcp2515IsReadyToSend(uint8_t status)
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
modm::ResumableResult<bool>
modm::Mcp2515<SPI, CS, INT>::mcp2515SendMessage(const can::Message &message)
{
	using namespace modm::mcp2515;

	RF_BEGIN();

	statusBufferS = RF_CALL(readStatus(READ_STATUS));

	addressBufferS = static_cast<uint8_t>(false);

	// /// send if ready, else return that nothing was sent
	if (mcp2515IsReadyToSend(statusBufferS))
	{
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

		if (addressBufferS == 0x00 || addressBufferS == 0x02 || addressBufferS == 0x04)
		{
			RF_WAIT_UNTIL(this->acquireMaster());
			chipSelect.reset();

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
			RF_CALL(spi.transfer(tx_buf, rx_buf, 6 + message.length));

			// RF_WAIT_UNTIL(spi.getDmaState());
			chipSelect.set();
			delayS.restart(100ns);
			RF_WAIT_UNTIL(delayS.isExpired());

			// chipSelect.reset();
			// RF_CALL(spi.transfer(WRITE_TX | addressBufferS));
			// chipSelect.set();

			// send message via RTS command
			chipSelect.reset();
			addressBufferS = (addressBufferS == 0) ? 1 : addressBufferS;  // 0 2 4 => 1 2 4
			RF_CALL(spi.transfer(RTS | addressBufferS));
			chipSelect.set();
			RF_WAIT_UNTIL(this->releaseMaster());
		}
	}

	RF_END_RETURN(static_cast<bool>(addressBufferS));
}

// ----------------------------------------------------------------------------

template<typename SPI, typename CS, typename INT>
void
modm::Mcp2515<SPI, CS, INT>::writeRegister(uint8_t address, uint8_t data)
{
	chipSelect.reset();
	spi.transferBlocking(WRITE);
	spi.transferBlocking(address);
	spi.transferBlocking(data);
	chipSelect.set();
}

template<typename SPI, typename CS, typename INT>
uint8_t
modm::Mcp2515<SPI, CS, INT>::readRegister(uint8_t address)
{
	chipSelect.reset();
	spi.transferBlocking(READ);
	spi.transferBlocking(address);
	uint8_t data = spi.transferBlocking(0xff);
	chipSelect.set();
	return data;
}

template<typename SPI, typename CS, typename INT>
void
modm::Mcp2515<SPI, CS, INT>::bitModify(uint8_t address, uint8_t mask, uint8_t data)
{
	chipSelect.reset();
	spi.transferBlocking(BIT_MODIFY);
	spi.transferBlocking(address);
	spi.transferBlocking(mask);
	spi.transferBlocking(data);
	chipSelect.set();
}

template<typename SPI, typename CS, typename INT>
modm::ResumableResult<uint8_t>
modm::Mcp2515<SPI, CS, INT>::readStatus(uint8_t type)
{
	RF_BEGIN();

	RF_WAIT_UNTIL(this->acquireMaster());
	chipSelect.reset();

	tx_buf[0] = type;
	tx_buf[1] = 0xFF;
	// RF_CALL(spi.transfer(type));
	RF_CALL(spi.transfer(tx_buf, rx_buf, 2));
	chipSelect.set();

	statusBuffer = rx_buf[1];

	RF_WAIT_UNTIL(this->releaseMaster());
	RF_END_RETURN(statusBuffer);
}

// ----------------------------------------------------------------------------

template<typename SPI, typename CS, typename INT>
modm::ResumableResult<void>
modm::Mcp2515<SPI, CS, INT>::writeIdentifier(const uint32_t &identifier, bool isExtendedFrame)
{
	using namespace mcp2515;
	const uint32_t *ptr = &identifier;

	RF_BEGIN();

	// RF_WAIT_UNTIL(this->acquireMaster());
	if (isExtendedFrame)
	{
		RF_CALL(spi.transfer(*((uint16_t *)ptr + 1) >> 5));

		// calculate the next values
		a = (*((uint8_t *)ptr + 2) << 3) & 0xe0;
		a |= MCP2515_IDE;
		a |= (*((uint8_t *)ptr + 2)) & 0x03;
		RF_CALL(spi.transfer(a));
		RF_CALL(spi.transfer(*((uint8_t *)ptr + 1)));
		RF_CALL(spi.transfer(*((uint8_t *)ptr)));
	} else
	{
		RF_CALL(spi.transfer(*((uint16_t *)ptr) >> 3));
		RF_CALL(spi.transfer(*((uint8_t *)ptr) << 5));
		RF_CALL(spi.transfer(0));
		RF_CALL(spi.transfer(0));
	}
	// RF_WAIT_UNTIL(this->releaseMaster());

	RF_END();
}

template<typename SPI, typename CS, typename INT>
modm::ResumableResult<void>
modm::Mcp2515<SPI, CS, INT>::writeIdentifierBuffer(const uint32_t &identifier, bool isExtendedFrame,
												   uint8_t *buffer, uint8_t start_index)
{
	using namespace mcp2515;
	const uint32_t *ptr = &identifier;

	RF_BEGIN();

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
	// RF_WAIT_UNTIL(this->releaseMaster());

	RF_END();
}

template<typename SPI, typename CS, typename INT>
modm::ResumableResult<bool>
modm::Mcp2515<SPI, CS, INT>::readIdentifier(uint32_t &identifier)
{
	using namespace mcp2515;
	const uint32_t *ptr = &identifier;

	RF_BEGIN();

	// RF_WAIT_UNTIL(this->acquireMaster());
	a = RF_CALL(spi.transfer(0xff));
	b = RF_CALL(spi.transfer(0xff));

	readIdentifierSuccessfulFlag = false;

	if (b & MCP2515_IDE)
	{
		*((uint16_t *)ptr + 1) = (uint16_t)a << 5;
		*((uint8_t *)ptr + 1) = RF_CALL(spi.transfer(0xff));

		*((uint8_t *)ptr + 2) |= (b >> 3) & 0x1C;
		*((uint8_t *)ptr + 2) |= b & 0x03;

		*((uint8_t *)ptr) = RF_CALL(spi.transfer(0xff));

		readIdentifierSuccessfulFlag = true;
	} else
	{
		RF_CALL(spi.transfer(0xff));

		*((uint8_t *)ptr + 3) = 0;
		*((uint8_t *)ptr + 2) = 0;

		*((uint16_t *)ptr) = (uint16_t)a << 3;

		RF_CALL(spi.transfer(0xff));

		*((uint8_t *)ptr) |= b >> 5;
	}
	// RF_WAIT_UNTIL(this->releaseMaster());

	RF_END_RETURN(readIdentifierSuccessfulFlag);
}
