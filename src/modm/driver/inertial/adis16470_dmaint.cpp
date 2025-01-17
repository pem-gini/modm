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

#include "adis16470_dmaint.hpp"

modm::IOStream&
modm::operator << (modm::IOStream& os, const adis16470DmaInt::DiagStat_t& c) {
	os << "DiagStat(";
	if(c & adis16470DmaInt::DiagStat::ClockError)
		os << "ClockError ";
	if(c & adis16470DmaInt::DiagStat::MemoryFailure)
		os << "MemoryFailure ";
	if(c & adis16470DmaInt::DiagStat::SensorFailure)
		os << "SensorFailure ";
	if(c & adis16470DmaInt::DiagStat::StandbyMode)
		os << "StandbyMode ";
	if(c & adis16470DmaInt::DiagStat::SpiCommunicationError)
		os << "SpiCommunicationError ";
	if(c & adis16470DmaInt::DiagStat::FlashUpdateFailure)
		os << "FlashUpdateFailure ";
	if(c & adis16470DmaInt::DiagStat::DataPathOverrun)
		os << "DataPathOverrun ";
	os << ")";
	return os;
}

modm::IOStream&
modm::operator << (modm::IOStream& os, const adis16470DmaInt::MscCtrl_t& c) {
	os << "MscCtrl(";
	if(c & adis16470DmaInt::MscCtrl::LinearGCompensationGyro)
		os << "LinearGCompensationGyro ";
	if(c & adis16470DmaInt::MscCtrl::PointOfPercussionAlign)
		os << "PointOfPercussionAlign ";
	os << "SyncFunction=";
	switch (adis16470DmaInt::SyncFunction_t::get(c))
	{
	case adis16470DmaInt::SyncFunction::PulseSync:
		os << "PulseSync ";
		break;
	case adis16470DmaInt::SyncFunction::OutputSync:
		os << "OutputSync ";
		break;
	case adis16470DmaInt::SyncFunction::ScaledSync:
		os << "ScaledSync ";
		break;
	case adis16470DmaInt::SyncFunction::DirectSync:
		os << "DirectSync ";
		break;
	case adis16470DmaInt::SyncFunction::InternalClock:
		os << "InternalClock ";
		break;
	default:
		os << "? ";
		break;
	}
	if(c & adis16470DmaInt::MscCtrl::SyncPolarity)
		os << "SyncPolarity ";
	if(c & adis16470DmaInt::MscCtrl::DrPolarity)
		os << "DrPolarity ";
	os << ")";
	return os;
}