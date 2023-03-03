// coding: utf-8
/*
 * Copyright (c) 2022, Raphael Lehmann
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#ifndef MODM_ADIS16470_DMAINT_HPP
	#error	"Don't include this file directly, use 'adis16470.hpp' instead!"
#endif

namespace modm
{

template<class SpiQueuedDma, class Cs>
modm::ResumableResult<void>
Adis16470DmaInt<SpiQueuedDma, Cs>::initialize()
{
	RF_BEGIN();
	this->attachConfigurationHandler([]() {
		configuration.pre.call_if();
	});
	Cs::setOutput(modm::Gpio::High);
	timeout.restart(tStall);
	RF_END();
}


template<class SpiQueuedDma, class Cs>
modm::ResumableResult<std::optional<uint16_t>>
Adis16470DmaInt<SpiQueuedDma, Cs>::readRegister(Register reg)
{
	RF_BEGIN();

	RF_CALL(spi.disable());

	if (getRegisterAccess(reg) == AccessMethod::Write) {
		// Reading this register is not permitted
		RF_RETURN(std::nullopt);
	}

	// Ensure CS was not asserted for T_stall
	RF_WAIT_UNTIL(timeout.isExpired());

	RF_WAIT_UNTIL(this->acquireMaster());
	Cs::reset();

	buffer[0] = uint8_t(reg) & 0b0111'1111;
	buffer[1] = 0;
	RF_CALL(SpiQueuedDma::transfer(buffer.data(), nullptr, 2));

	if (this->releaseMaster()) {
		Cs::set();
	}
	timeout.restart(tStall);
	RF_WAIT_UNTIL(timeout.isExpired());
	RF_WAIT_UNTIL(this->acquireMaster());
	Cs::reset();

	RF_CALL(SpiQueuedDma::transfer(buffer.data(), &buffer[2], 2));

	if (this->releaseMaster()) {
		Cs::set();
	}
	timeout.restart(tStall);

	RF_CALL(spi.enable());

	RF_END_RETURN((static_cast<uint16_t>(buffer[2]) << 8) | buffer[3]);
}

template<class SpiQueuedDma, class Cs>
modm::ResumableResult<modm::adis16470DmaInt::DiagStat_t>
Adis16470DmaInt<SpiQueuedDma, Cs>::readDiagStat()
{
	RF_BEGIN();
	tmp = RF_CALL(readRegister(Register::DIAG_STAT));
	RF_END_RETURN(DiagStat_t(*tmp));
}

template<class SpiQueuedDma, class Cs>
modm::ResumableResult<modm::adis16470DmaInt::MscCtrl_t>
Adis16470DmaInt<SpiQueuedDma, Cs>::readMscCtrl()
{
	RF_BEGIN();
	tmp = RF_CALL(readRegister(Register::MSC_CTRL));
	RF_END_RETURN(MscCtrl_t(*tmp));
}

template<class SpiQueuedDma, class Cs>
modm::ResumableResult<bool>
Adis16470DmaInt<SpiQueuedDma, Cs>::writeRegister(Register reg, uint16_t value)
{
	RF_BEGIN();

	RF_CALL(spi.disable());

	if (getRegisterAccess(reg) == AccessMethod::Read) {
		// Writing to this register is not permitted
		RF_RETURN(false);
	}

	// Ensure CS was not asserted for T_stall
	RF_WAIT_UNTIL(timeout.isExpired());

	RF_WAIT_UNTIL(this->acquireMaster());
	Cs::reset();

	buffer[0] = (uint8_t(reg) & 0b0111'1111) | 0b1000'0000;
	buffer[1] = static_cast<uint8_t>(value);
	RF_CALL(SpiQueuedDma::transfer(buffer.data(), nullptr, 2));

	if (this->releaseMaster()) {
		Cs::set();
	}
	timeout.restart(tStall);
	RF_WAIT_UNTIL(timeout.isExpired());
	RF_WAIT_UNTIL(this->acquireMaster());
	Cs::reset();

	buffer[0] = ((uint8_t(reg) + 1) & 0b0111'1111) | 0b1000'0000;
	buffer[1] = static_cast<uint8_t>(value >> 8);
	RF_CALL(SpiQueuedDma::transfer(buffer.data(), nullptr, 2));

	if (this->releaseMaster()) {
		Cs::set();
	}
	timeout.restart(tStall);

	RF_CALL(spi.enable());

	RF_END_RETURN(true);
}

template<class SpiQueuedDma, class Cs>
modm::ResumableResult<void>
Adis16470DmaInt<SpiQueuedDma, Cs>::writeMscCtrl(modm::adis16470DmaInt::MscCtrl_t value)
{
	RF_BEGIN();

	RF_CALL(writeRegister(Register::MSC_CTRL, value.value));

	// Writing to MSC_CTRL take approx. 3ms
	timeout.restart(std::chrono::milliseconds(3));

	RF_END();
}

template<class SpiQueuedDma, class Cs>
modm::ResumableResult<void>
Adis16470DmaInt<SpiQueuedDma, Cs>::writeGlobCmd(modm::adis16470DmaInt::GlobCmd_t value)
{
	RF_BEGIN();
	RF_CALL(writeRegister(Register::GLOB_CMD, value.value));
	RF_END();
}

template<class SpiQueuedDma, class Cs>
template<frequency_t frequency, percent_t tolerance>
modm::ResumableResult<void>
Adis16470DmaInt<SpiQueuedDma, Cs>::setDataOutputFrequency()
{
	// Output data rate R = 2000SPS / (DEC_RATE + 1)
	constexpr uint16_t decRate = ((2000 / frequency) - 1) & 0b111'1111'1111;
	constexpr uint16_t actualFrequency = 2000 / (decRate + 1);

	static_assert(frequency < 2000, "Maximum data output rate is 2000Hz");
	modm::PeripheralDriver::assertBaudrateInTolerance<actualFrequency, frequency, tolerance>();

	RF_BEGIN();
	RF_CALL(writeRegister(Register::DEC_RATE, decRate));
	RF_END();
}


template<class SpiQueuedDma, class Cs>
modm::ResumableResult<bool>
Adis16470DmaInt<SpiQueuedDma, Cs>::readRegisterSequence(std::span<const Register> sequence, std::span<uint16_t> values)
{
	RF_BEGIN();

	RF_CALL(spi.disable());

	if(sequence.size() != values.size()) {
		// Mismatching std::span sizes
		RF_RETURN(false);
	}

	// Ensure CS was not asserted for T_stall
	RF_WAIT_UNTIL(timeout.isExpired());

	RF_WAIT_UNTIL(this->acquireMaster());
	Cs::reset();

	for (i = 0; i < sequence.size(); i++) {
		if (getRegisterAccess(sequence[i]) == AccessMethod::Write) {
			// Reading this register is not permitted
			if (this->releaseMaster()) {
				Cs::set();
			}
			timeout.restart(tStall);
			RF_RETURN(false);
		}

		buffer[0] = uint8_t(sequence[i]) & 0b0111'1111;
		buffer[1] = 0;
		RF_CALL(SpiQueuedDma::transfer(buffer.data(), &buffer[2], 2));

		if (this->releaseMaster()) {
			Cs::set();
		}

		timeout.restart(tStall);
		RF_WAIT_UNTIL(timeout.isExpired());

		RF_WAIT_UNTIL(this->acquireMaster());
		Cs::reset();

		if (i != 0) {
			values[i-1] = (static_cast<uint16_t>(buffer[2]) << 8) | buffer[3];
		}
	}

	// one additionl transfer to retrieve value of last register
	buffer[0] = uint8_t(sequence[0]) & 0b0111'1111;
	buffer[1] = 0;
	RF_CALL(SpiQueuedDma::transfer(buffer.data(), &buffer[2], 2));
	values[values.size()-1] = (static_cast<uint16_t>(buffer[2]) << 8) | buffer[3];

	if (this->releaseMaster()) {
		Cs::set();
	}
	timeout.restart(tStall);

	RF_CALL(spi.enable());

	RF_END_RETURN(true);
}

template<class SpiQueuedDma, class Cs>
modm::ResumableResult<bool>
Adis16470DmaInt<SpiQueuedDma, Cs>::readRegisterBurst(std::array<uint16_t, 11>& data)
{
	RF_BEGIN();

	RF_CALL(spi.disable());

	buffer.fill(0);
	buffer[0] = 0x68;

	RF_WAIT_UNTIL(this->acquireMaster());
	Cs::reset();

	RF_CALL(SpiQueuedDma::transfer(buffer.data(), reinterpret_cast<uint8_t*>(data.data()), 22));

	if (this->releaseMaster()) {
		Cs::set();
	}
	timeout.restart(tStall);

	// Calulate checksum
	checksum = 0;
	for (i = 0; i < 18; i++) {
		checksum += reinterpret_cast<uint8_t*>(data.data() + 1)[i];
	}

	// Fix endianness
	for (i = 1; i < 11; i++) {
		data[i] = modm::fromBigEndian(data[i]);
	}

	RF_CALL(spi.enable());

	RF_END_RETURN(checksum == data[10]);
}

template<class SpiQueuedDma, class Cs>
template<typename Int>
void
Adis16470DmaInt<SpiQueuedDma, Cs>::registerInterruptCallback(auto&& cb)
{
	intCallback = cb;
	modm::platform::Exti::enableInterrupts<Int>();
	modm::platform::Exti::connect<Int>(modm::platform::Exti::Trigger::RisingEdge, [&](uint8_t /*line*/) mutable {
		if(intCallback){
			intCallback();
		}
	});
}


template<class SpiQueuedDma, class Cs>
void 
Adis16470DmaInt<SpiQueuedDma, Cs>::readRegisterBurstIntoBuffer()
{
	auto post = [&](){
		// Calulate checksum
		checksum = 0;
		for (i = 0; i < 18; i++) {
			checksum += reinterpret_cast<uint8_t*>(burstData.buffer.data() + 1)[i];
		}
		// Fix endianness
		for (i = 1; i < 11; i++) {
			burstData.buffer[i] = modm::fromBigEndian(burstData.buffer[i]);
		}
		burstData.valid = checksum == burstData.buffer[10];
		if(registerBurstFinished){
			registerBurstFinished(getRegisterBurst());
		}
		return false;
	};

	buffer.fill(0);
	buffer[0] = 0x68;
	spi.pipeline(
		modm::SpiTransferStep{&buffer[0], reinterpret_cast<uint8_t*>(burstData.buffer.data()), sizeof(RegisterBurstBuffer), post, nullptr, configuration, modm::CsBehavior<Cs>(ChipSelect::TOGGLE)}
	);
}

} // namespace modm
