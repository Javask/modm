/*
 * Copyright (c) 2011, Fabian Greif
 * Copyright (c) 2013, Kevin Läufer
 * Copyright (c) 2013-2017, Niklas Hauser
 * Copyright (c) 2014, 2016, Sascha Schade
 * Copyright (c) 2022, Andrey Kunitsyn
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#include <modm/board.hpp>
#include <modm/debug/logger.hpp>

// ----------------------------------------------------------------------------
// Set the log level
#undef MODM_LOG_LEVEL
#define MODM_LOG_LEVEL modm::log::INFO

// Create an IODeviceWrapper around the Uart Peripheral we want to use
modm::IODeviceWrapper<Uart0, modm::IOBuffer::BlockIfFull> loggerDevice;

// Set all four logger streams to use the UART
modm::log::Logger modm::log::debug(loggerDevice);
modm::log::Logger modm::log::info(loggerDevice);
modm::log::Logger modm::log::warning(loggerDevice);
modm::log::Logger modm::log::error(loggerDevice);

// ----------------------------------------------------------------------------
int
main()
{
	Board::initialize();

	// initialize Uart0 for MODM_LOG_*
	Uart0::connect<GpioOutput0::Tx>();
	Uart0::initialize<Board::SystemClock, 115200_Bd>();

	// Use the logging streams to print some messages.
	// Change MODM_LOG_LEVEL above to enable or disable these messages
	MODM_LOG_DEBUG << "debug" << modm::endl;
	MODM_LOG_INFO << "info" << modm::endl;
	MODM_LOG_WARNING << "warning" << modm::endl;
	MODM_LOG_ERROR << "error" << modm::endl;

	uint32_t uptime{};
	while (true)
	{
		Board::LedGreen::reset();
		modm::delay(100ms);

		Board::LedGreen::set();
		modm::delay(900ms);

		MODM_LOG_INFO << "Seconds since reboot: " << ++uptime << modm::endl;
	}

	return 0;
}
