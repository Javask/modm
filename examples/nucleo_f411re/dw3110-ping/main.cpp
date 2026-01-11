/*
 * Copyright (c) 2024, Elias H.
 * Copyright (c) 2024, Raphael Lehmann
 * Copyright (c) 2024,2026, Michael Jossen
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include <modm/board.hpp>
#include <modm/debug/logger.hpp>
#include <modm/driver/radio/dw3110/dw3110_phy.hpp>
#include <modm/processing.hpp>

using namespace Board;
using namespace std::chrono_literals;

using MySpiMaster = modm::platform::SpiMaster1;
using MyDw3110 = modm::Dw3110Phy<MySpiMaster, D10>;

class TXThread : public modm::Fiber<>
{
public:
	TXThread() : Fiber([this] { run(); }) {}

	bool
	init()
	{
		auto ret = radio.initialize(modm::Dw3110::Channel::Channel9,
									modm::Dw3110::PreambleCode::Code_64Mhz_9,
									modm::Dw3110::PreambleLength::Preamble_128,
									modm::Dw3110::StartFrameDelimiter::Decawave_8);
		radio.setEnableLongFrames(true);
		return ret;
	}

	size_t
	getCount()
	{
		return sentCount;
	}

private:
	bool
	run()
	{
		while (true)
		{
			txdata[txdata.size() - 1]++;
			modm::this_fiber::sleep_for(Button::read() ? 500ms : 100ms);

			if (radio.transmit(txdata, true) == MyDw3110::Error::None)
			{
				sentCount++;
			} else
			{
				MODM_LOG_DEBUG << "[TX] Failed to trasmit!" << modm::endl;
			}
		}
	}

	MyDw3110 radio{};
	std::array<uint8_t, 5> txdata = {0xBA, 0xDE, 0xAF, 0xFE, 0x00};
	modm::Timeout timeout{10ms};
	size_t sentCount{0};
} tx;

modm::Fiber fiber_report([] {
	while (true)
	{
		modm::this_fiber::sleep_for(1s);
		MODM_LOG_DEBUG << "Sent " << tx.getCount() << modm::endl;
	}
});

int
main()
{
	Board::initialize();
	LedD13::setOutput();

	MySpiMaster::initialize<Board::SystemClock, 24_MHz>();
	MySpiMaster::connect<GpioA6::Miso, GpioA7::Mosi, GpioA5::Sck>();

	// Use the logging streams to print some messages.
	// Change MODM_LOG_LEVEL above to enable or disable these messages
	MODM_LOG_DEBUG << "debug" << modm::endl;
	MODM_LOG_INFO << "info" << modm::endl;
	MODM_LOG_WARNING << "warning" << modm::endl;
	MODM_LOG_ERROR << "error" << modm::endl;

	MODM_LOG_INFO << "Initializing Devices..." << modm::endl;
	bool success = true;
	if (not tx.init())
	{
		MODM_LOG_ERROR << "Failed to initialize TX Device!" << modm::endl;
		success = false;
	}
	modm_assert(success, "user", "Failed to initialize devices!");

	MODM_LOG_INFO << "Starting ping pong..." << modm::endl;
	modm::fiber::Scheduler::run();

	return 0;
}
