/*
 * Copyright (c) 2024, Michael Jossen
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include <modm/board.hpp>
#include <modm/debug/logger.hpp>
#include <modm/driver/motion/paa5100je.hpp>
#include <modm/processing/timer.hpp>

using namespace std::chrono_literals;

// Setup logging to actually print values
#undef MODM_LOG_LEVEL
#define MODM_LOG_LEVEL modm::log::INFO

modm::IODeviceWrapper<Uart0, modm::IOBuffer::BlockIfFull> loggerDevice;

modm::log::Logger modm::log::debug(loggerDevice);
modm::log::Logger modm::log::info(loggerDevice);
modm::log::Logger modm::log::warning(loggerDevice);
modm::log::Logger modm::log::error(loggerDevice);

using MySpiMaster = modm::platform::SpiMaster0;
using MyPaa5100je = modm::Paa5100je<MySpiMaster, GpioOutput5>;

int
main()
{
	Board::initialize();

	Uart0::connect<GpioOutput0::Tx>();
	Uart0::initialize<Board::SystemClock, 115200_Bd>();

	MySpiMaster::connect<GpioOutput3::Tx, GpioInput4::Rx, GpioOutput2::Sclk>();
	MySpiMaster::initialize<Board::SystemClock, 2_MHz>();

	MyPaa5100je sensor{};
	RF_CALL_BLOCKING(sensor.initialize());
	MODM_LOG_INFO << "Initialized device." << modm::endl;

	uint8_t prod = RF_CALL_BLOCKING(sensor.getProductId());
	MODM_LOG_INFO << "ProductId: 0x" << modm::hex << prod << modm::endl;

	modm::PeriodicTimer timer{50ms};
	modm::Timeout retryTimeout{100ms};
	MyPaa5100je::Motion2D data{};
	int32_t x{}, y{};

	while (true)
	{
		if (timer.execute())
		{
			retryTimeout.restart();
			bool res = RF_CALL_BLOCKING(sensor.getMotionData(data));
			while (!res && !retryTimeout.execute())
			{
				modm::ShortPreciseTimeout wait{1ms};
				while (!wait.execute()) { __NOP(); }
				res = RF_CALL_BLOCKING(sensor.getMotionData(data));
			}

			if (res)
			{
				x += data.x;
				y += data.y;
				MODM_LOG_INFO << "X: " << x << " Y: " << y << " dX: " << data.x << " dY: " << data.y
							  << modm::endl;
			}
		}
	}
	return 0;
}