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
// using SpiTransferConditional = etl::delegate<bool()>;
// using SpiTransferLength = etl::delegate<std::size_t()>;
using SpiTransferCallback = std::function<void()>;
using SpiTransferConditional = std::function<bool()>;
using SpiTransferLength = std::function<std::size_t()>;
using SpiTransferCs = std::function<void()>;

enum class ChipSelectBehavior{
	NONE,
	RESET,
	SET,
	TOGGLE
};

template <typename GPIO>
struct ChipSelectBehaviorWrapper{
	using CS = GPIO;
	ChipSelectBehavior behavior;
};
class SpiTransferChipSelectBehavior{
public:
	SpiTransferChipSelectBehavior(auto wrapper) :
		pre{[wrapper](){
			if constexpr (!std::is_void<typename decltype(wrapper)::CS>::value){
				switch(wrapper.behavior){
					case ChipSelectBehavior::NONE: break;
					case ChipSelectBehavior::RESET: decltype(wrapper)::CS::reset(); break;
					case ChipSelectBehavior::TOGGLE: decltype(wrapper)::CS::reset(); break;
					default: break;
				};
			}
		}},
		post{[wrapper](){
			if constexpr (!std::is_void<typename decltype(wrapper)::CS>::value){
				switch(wrapper.behavior){
					case ChipSelectBehavior::NONE: break;
					case ChipSelectBehavior::SET: decltype(wrapper)::CS::set(); break;
					case ChipSelectBehavior::TOGGLE: decltype(wrapper)::CS::set(); break;
					default: break;
				};
			}
		}}{}

	modm::SpiTransferCs pre;
	modm::SpiTransferCs post;
};

template<typename CS = void>
static constexpr SpiTransferChipSelectBehavior CsBehavior(ChipSelectBehavior behavior = ChipSelectBehavior::NONE){
	ChipSelectBehaviorWrapper<CS> wrapper{behavior};
	SpiTransferChipSelectBehavior cs = SpiTransferChipSelectBehavior(wrapper);
	return cs;
}

class SpiTransferStep{
public:
	SpiTransferStep(const uint8_t* tx_, uint8_t* rx_, auto length_, auto cb_, auto condition_,
					auto configuration_, SpiTransferChipSelectBehavior csbehavior = CsBehavior<void>())
		: tx{tx_},
		  rx{rx_},
		  cb{cb_},
		  condition{condition_},
		  configuration{configuration_},
		  cs{csbehavior}
	{
		if constexpr (std::is_integral_v<decltype(length_)>){
			length = [length_]() { return length_; };
		} else{
			length = length_;
		}
	}
	const uint8_t *tx;
	uint8_t *rx; 
	SpiTransferLength length;
	modm::SpiTransferCallback cb;
	modm::SpiTransferConditional condition;
	modm::SpiTransferConfiguration configuration;
	modm::SpiTransferChipSelectBehavior cs;
};

} // namespace modm

#endif // MODM_INTERFACE_SPI_HPP
