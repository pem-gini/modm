/*
 * Copyright (c) 2013, Kevin Läufer
 * Copyright (c) 2013-2017, Niklas Hauser
 * Copyright (c) 2016, Raphael Lehmann
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#include <modm/board.hpp>
#include <modm/platform/spi/spi_master_1.hpp>
#include <modm/platform/spi/spi_master_2.hpp>
#include <modm/processing/protothread.hpp>
#include <modm/driver/can/mcp2515.hpp>

#define SENDER 1

// Set the log level
#undef MODM_LOG_LEVEL
#define MODM_LOG_LEVEL modm::log::DEBUG

// If you use a different SPI instance, you may have to also choose different
// GPIOs to connect to.
using Cs = GpioOutputA4;
using Mosi = GpioOutputB5;
using Miso = GpioInputB4;
using Sck = GpioOutputB3;
using Int = GpioInputC7;
using SpiMaster = SpiMaster1;
// Note that you can also use a bit-banged SPI driver as a drop-in replacement
// using SpiMaster = BitBangSpiMaster<Sck, Mosi, Miso>;

// Default filters to receive any extended CAN frame
FLASH_STORAGE(uint8_t canFilter[]) = {
	MCP2515_FILTER_EXTENDED(0),  // Filter 0
	MCP2515_FILTER_EXTENDED(0),  // Filter 1

	MCP2515_FILTER_EXTENDED(0),  // Filter 2
	MCP2515_FILTER_EXTENDED(0),  // Filter 3
	MCP2515_FILTER_EXTENDED(0),  // Filter 4
	MCP2515_FILTER_EXTENDED(0),  // Filter 5

	MCP2515_FILTER_EXTENDED(0),  // Mask 0
	MCP2515_FILTER_EXTENDED(0),  // Mask 1
};

modm::Mcp2515<SpiMaster, Cs, Int> mcp2515;

class MyTask : modm::pt::Protothread
{
public:
	MyTask() :
		message_{0x1337},
		i_{0},
		j_{0}
		{}

	bool
	run()
	{
		PT_BEGIN();

		MODM_LOG_INFO << "Initializing mcp2515 ..." << modm::endl;
		// Configure MCP2515 and set the filters
		initialized_ = mcp2515.initialize<8_MHz, 500_kbps>();
		MODM_LOG_INFO << "Success: " << initialized_ << modm::endl;
		mcp2515.setFilter(modm::accessor::asFlash(canFilter));
		MODM_LOG_INFO << "Running ... " << modm::endl;
#if SENDER == 0
		while (initialized_)
		{
			// receive messages
			if (mcp2515.isMessageAvailable())
			{
				MODM_LOG_INFO << "Message Available ... " << modm::endl;
				mcp2515.getMessage(message_);
				for(i_ = 0; i_ < message_.length; ++i_){
					MODM_LOG_INFO << modm::hex<< " 0x" << message_.data[i_];
				}
				MODM_LOG_INFO << modm::endl;
				MODM_LOG_INFO << "Received message [" << j_ << "]: " << modm::hex << message_.identifier << modm::endl;
				j_+=1;
				MODM_LOG_INFO << modm::endl;
			}
			PT_CALL(mcp2515.update());
		}
#else
		while (initialized_)
		{
			wait_.restart(1000ms);
			PT_WAIT_UNTIL(wait_.isExpired());
			message_.length = 2;
			message_.data[0] = i_++;
			message_.data[1] = i_++;
			MODM_LOG_INFO << "Sending Message ... "<< modm::endl;
			for(j_ = 0; j_ < message_.length; ++j_){
				MODM_LOG_INFO << modm::hex<< " 0x" << message_.data[j_];
			}
			MODM_LOG_INFO << modm::endl;
			mcp2515.sendMessage(message_);
			PT_CALL(mcp2515.update());
		}
#endif

		PT_END();
	}

private:
	modm::can::Message message_;
	bool initialized_;
	modm::ShortTimeout wait_;
	uint8_t i_, j_;
};

MyTask task;

int
main()
{
	Board::initialize();

	MODM_LOG_INFO << "Hello" << modm::endl;
	// Initialize SPI interface and the other pins
	// needed by the MCP2515
	SpiMaster::connect<Miso::Miso, Mosi::Mosi, Sck::Sck>();
	/// we initialize a higher baud rate then n the avr example, dunnow hats the mcp2515 is capable
	/// of
	SpiMaster::initialize<Board::SystemClock, 20_MHz>();
	Cs::setOutput();
	Int::setInput(Gpio::InputType::PullUp);

	while (true) {
		task.run();
	}
}