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
#include <modm/platform/exti/exti.hpp>
#include <modm/processing.hpp>

using namespace Board;
using namespace std::chrono_literals;

using MySpiMaster = modm::platform::SpiMaster1;
using MyDw3110 = modm::Dw3110Phy<MySpiMaster, D10>;
using MyDw3110IRQPin = D8;

class RXThread : public modm::Fiber<>
{
public:
	RXThread() : Fiber([this] { run(); }) {}

	bool
	init()
	{
		auto ret = radio.initialize(modm::Dw3110::Channel::Channel9,
									modm::Dw3110::PreambleCode::Code_64Mhz_9,
									modm::Dw3110::PreambleLength::Preamble_128,
									modm::Dw3110::StartFrameDelimiter::Decawave_8);
		radio.setEnableLongFrames(true);
		radio.setDoubleBuffering(true);
		modm::Dw3110::SystemStatus_t enabledIRQs = {};

		// Errors
		enabledIRQs.set(modm::Dw3110::SystemStatus::RXPHE);
		enabledIRQs.set(modm::Dw3110::SystemStatus::RXFCE);
		enabledIRQs.set(modm::Dw3110::SystemStatus::RXFSL);

		// Success
		enabledIRQs.set(modm::Dw3110::SystemStatus::RXFR);

		// Timeout
		enabledIRQs.set(modm::Dw3110::SystemStatus::RXFTO);
		enabledIRQs.set(modm::Dw3110::SystemStatus::RXSTO);
		enabledIRQs.set(modm::Dw3110::SystemStatus::RXPTO);

		radio.setInterruptsEnabled(enabledIRQs);
		return ret;
	}

	void
	notifyPacket()
	{
		packetRdy = true;
	}

	size_t
	getCount1()
	{
		return recvCount_1;
	}

	size_t
	getCount2()
	{
		return recvCount_2;
	}

private:
	bool
	run()
	{
		while (true)
		{
			MyDw3110::RXBuffer_t rdy_pkts;
			while (not radio.packetReady(&rdy_pkts))
			{
				modm::this_fiber::poll([this]() {
					if (not radio.isReceiving()) { radio.startReceive(); }
					return packetRdy;
				});
				packetRdy = false;
			}
			if (rdy_pkts.any(MyDw3110::RXBuffer::RX_BUFFER_0))
			{
				if (radio.fetchPacket(rxdata, rxlen, MyDw3110::RXBuffer::RX_BUFFER_0))
					recvCount_1++;
				radio.releaseRXBuffer();
			}
			if (rdy_pkts.any(MyDw3110::RXBuffer::RX_BUFFER_1))
			{
				if (radio.fetchPacket(rxdata, rxlen, MyDw3110::RXBuffer::RX_BUFFER_1))
					recvCount_2++;
				radio.releaseRXBuffer();
			}
		}
	}
	bool packetRdy = false;
	constexpr static size_t RxBufferSize = 1021;  // Maximum supported packet size
	MyDw3110 radio{};
	size_t rxlen{0}, recvCount_1{0}, recvCount_2;
	std::array<uint8_t, RxBufferSize> rxdata = {};
} rx;

modm::Fiber fiber_report([] {
	while (true)
	{
		modm::this_fiber::sleep_for(1s);
		MODM_LOG_DEBUG << "Received in Buffers 1:" << rx.getCount1() << " 2:" << rx.getCount2()
					   << modm::endl;
	}
});

int
main()
{
	Board::initialize();
	LedD13::setOutput();

	MyDw3110IRQPin::setInput(modm::platform::Gpio::InputType::PullDown);
	modm::platform::Exti::connect<MyDw3110IRQPin>(modm::platform::Exti::Trigger::RisingEdge,
												  [](uint8_t) { rx.notifyPacket(); });

	MySpiMaster::initialize<Board::SystemClock, 21_MHz>();
	MySpiMaster::connect<D12::Miso, D11::Mosi, D13::Sck>();

	// Use the logging streams to print some messages.
	// Change MODM_LOG_LEVEL above to enable or disable these messages
	MODM_LOG_DEBUG << "debug" << modm::endl;
	MODM_LOG_INFO << "info" << modm::endl;
	MODM_LOG_WARNING << "warning" << modm::endl;
	MODM_LOG_ERROR << "error" << modm::endl;

	MODM_LOG_INFO << "Initializing Devices..." << modm::endl;
	bool success = true;
	if (not rx.init())
	{
		MODM_LOG_ERROR << "Failed to initialize TR Device!" << modm::endl;
		success = false;
	}
	modm_assert(success, "user", "Failed to initialize devices!");

	MODM_LOG_INFO << "Starting ping pong..." << modm::endl;
	modm::fiber::Scheduler::run();

	return 0;
}
