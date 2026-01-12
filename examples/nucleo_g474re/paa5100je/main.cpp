// coding: utf-8
/*
 * Copyright (c) 2026, Michael Jossen
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include <modm/board.hpp>
#include <modm/driver/motion/paa5100je.hpp>
#include <modm/processing/fiber.hpp>

using namespace std::chrono_literals;
using namespace Board;

using MySpiMaster = modm::platform::SpiMaster1;
using MyPaa5100je = modm::Paa5100je<MySpiMaster, D10>;

constexpr modm::PreciseClock::duration timeout = 100ms;
MyPaa5100je sensor{};
MyPaa5100je::Motion2D data{};
int32_t x{}, y{};

modm::Fiber sensorThread([] {
	while (true)
	{
		modm::this_fiber::sleep_for(50ms);
		bool res = sensor.getMotionData(data);
		auto start = modm::PreciseClock::now();

		while (!res && modm::PreciseClock::now() - start < timeout)
		{
			res = sensor.getMotionData(data);
			modm::this_fiber::sleep_for(1ms);
		}

		if (res)
		{
			x += data.x;
			y += data.y;
			MODM_LOG_INFO << "X: " << x << " Y: " << y << " dX: " << data.x << " dY: " << data.y
						  << modm::endl;
		}
	}
});

int
main()
{
	Board::initialize();

	MySpiMaster::connect<D12::Miso, D11::Mosi, D13::Sck>();
	MySpiMaster::initialize<Board::SystemClock, 21_MHz>();

	const uint8_t prod = sensor.getProductId();
	MODM_LOG_INFO << "ProductId: 0x" << modm::hex << prod << modm::endl;
	modm_assert(sensor.initialize(), "user", "Failed to initialize device!");
	MODM_LOG_INFO << "Initialized device." << modm::endl;

	modm::fiber::Scheduler::run();
	return 0;
}