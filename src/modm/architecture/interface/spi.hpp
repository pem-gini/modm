/*
 * Copyright (c) 2009-2012, Fabian Greif
 * Copyright (c) 2010, Martin Rosekeit
 * Copyright (c) 2012-2015, Niklas Hauser
 * Copyright (c) 2013, Sascha Schade
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#ifndef MODM_INTERFACE_SPI_HPP
#define MODM_INTERFACE_SPI_HPP

#include <modm/architecture/interface/peripheral.hpp>

#include <etl/queue.h>
#include <etl/delegate.h>
#include <functional>

#include <modm/debug.hpp>

namespace modm
{

/// @ingroup modm_architecture_spi
struct Spi
{
	/// The signature of the configuration function.
	using ConfigurationHandler = void(*)();

	/// Spi Data Mode, Mode0 is the most common mode
	enum class
	DataMode : uint8_t
	{
		Mode0 = 0b00,	///< clock normal,   sample on rising  edge
		Mode1 = 0b01,	///< clock normal,   sample on falling edge
		Mode2 = 0b10,	///< clock inverted, sample on falling edge
		Mode3 = 0b11,	///< clock inverted, sample on rising  edge
	};

	/// Spi Data Order, MsbFirst is the most common mode
	enum class
	DataOrder : uint8_t
	{
		MsbFirst = 0b0,
		LsbFirst = 0b1,
	};
};



struct SpiTransferConfiguration{
	etl::delegate<void()> pre;
	etl::delegate<void()> post;
};
using SpiTransferCallback = etl::delegate<void()>;
using SpiTransferConditional = etl::delegate<bool()>;
// using SpiTransferLength = etl::delegate<std::size_t()>;
using SpiTransferLength = std::function<std::size_t()>;

class SpiTransferStep{
public:
	SpiTransferStep(const uint8_t* tx_, uint8_t* rx_, auto length_, auto cb_, auto condition_, auto configuration_){
		tx = tx_;
		rx = rx_;
		if constexpr(std::is_integral_v<decltype(length_)>){
			length = [length_](){return length_;};
		}
		if constexpr(!std::is_same_v<std::remove_reference_t<decltype(cb_)>, decltype(nullptr)>){
			cb = cb_;	
		}
		if constexpr(!std::is_same_v<std::remove_reference_t<decltype(condition_)>, decltype(nullptr)>){
			condition = condition_;	
		}
		if constexpr(!std::is_same_v<std::remove_reference_t<decltype(configuration_)>, decltype(nullptr)>){
			configuration_ = configuration_;	
		}
		MODM_LOG_INFO << "STEP" << modm::endl;
		MODM_LOG_INFO << "    - LENGTH: " << length() << modm::endl;
		MODM_LOG_INFO << "    - CB: " << cb.is_valid() << modm::endl;
		MODM_LOG_INFO << "    - COND: " << condition.is_valid() << modm::endl;
		MODM_LOG_INFO << "    - CFG pre: " << configuration.pre.is_valid() << modm::endl;
		MODM_LOG_INFO << "    - CFG post: " << configuration.post.is_valid() << modm::endl;
	}

	const uint8_t *tx;
	uint8_t *rx; 
	SpiTransferLength length;
	modm::SpiTransferCallback cb;
	modm::SpiTransferConditional condition;
	modm::SpiTransferConfiguration configuration;
};

} // namespace modm

#endif // MODM_INTERFACE_SPI_HPP
