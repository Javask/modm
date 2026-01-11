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
#include <modm/ui/button_group.hpp>

using namespace Board;
using namespace std::chrono_literals;

using MySpiMaster = modm::platform::SpiMaster1;
using MyDw3110 = modm::Dw3110Phy<MySpiMaster, D10>;
using MyDw3110IRQPin = D8;

class RadioThread : public modm::Fiber<>
{
public:
	RadioThread() : Fiber([this] { run(); }) {}

	bool
	initialize()
	{
		if (!radio.initialize(modm::Dw3110::Channel::Channel9,
							  modm::Dw3110::PreambleCode::Code_64Mhz_9,
							  modm::Dw3110::PreambleLength::Preamble_128,
							  modm::Dw3110::StartFrameDelimiter::Decawave_8))
		{
			return false;
		}
		radio.setDoubleBuffering(true);
		radio.setEnableLongFrames(true);

		MyDw3110IRQPin::setInput(modm::platform::Gpio::InputType::PullDown);
		modm::platform::Exti::connect<MyDw3110IRQPin>(modm::platform::Exti::Trigger::RisingEdge,
													  [this](uint8_t) { notifyPacket(); });

		// Trigger on packets
		modm::Dw3110::SystemStatus_t enabledIRQs = {};

		// Errors
		enabledIRQs.set(modm::Dw3110::SystemStatus::RXPHE);
		enabledIRQs.set(modm::Dw3110::SystemStatus::RXFCE);
		enabledIRQs.set(modm::Dw3110::SystemStatus::RXFSL);

		// Receive Success
		enabledIRQs.set(modm::Dw3110::SystemStatus::RXFR);

		// Timeout
		enabledIRQs.set(modm::Dw3110::SystemStatus::RXFTO);
		enabledIRQs.set(modm::Dw3110::SystemStatus::RXSTO);
		enabledIRQs.set(modm::Dw3110::SystemStatus::RXPTO);

		radio.setInterruptsEnabled(enabledIRQs);

		return true;
	}

	void
	notifyPacket()
	{
		packetrdy = true;
	}
	void
	notifyButton()
	{
		buttonPressed = true;
	}

	void
	printStatistics()
	{
		MODM_LOG_INFO << "Received " << rxcount << ", transmitted " << txcount << " packets."
					  << modm::endl;
	}

private:
	bool
	hasEvent()
	{
		return packetrdy || buttonPressed;
	}

	bool
	run()
	{
		if (!radio.startReceive()) { MODM_LOG_ERROR << "Failed to start receive!" << modm::endl; }

		MyDw3110::RXBuffer_t rdy_buffers;
		while (true)
		{
			// Wait until packet is rdy
			modm::this_fiber::poll(std::bind(&RadioThread::hasEvent, this));
			while (radio.packetReady(&rdy_buffers))
			{
				packetrdy = false;
				// Restart receive
				if (!radio.startReceive())
				{
					MODM_LOG_ERROR << "Failed to start receive!" << modm::endl;
				}
				// Process packet
				MyDw3110::RXBuffer buffer = MyDw3110::RXBuffer::RX_BUFFER_0;

				if (rdy_buffers.all(MyDw3110::RXBuffer::RX_BUFFER_0))
				{
					buffer = MyDw3110::RXBuffer::RX_BUFFER_0;
				} else if (rdy_buffers.all(MyDw3110::RXBuffer::RX_BUFFER_1))
				{
					buffer = MyDw3110::RXBuffer::RX_BUFFER_1;
				} else
				{
					MODM_LOG_ERROR << "Failed to determine which packet is ready!" << modm::endl;
					MODM_LOG_DEBUG << "Buffer Val: 0x" << modm::hex << (int)rdy_buffers.value
								   << modm::endl;
					break;
				}
				if (!radio.fetchPacket(rxdata, rxlen, buffer))
				{
					MODM_LOG_ERROR << "Failed to fetch packet!" << modm::endl;
				} else
				{
					rxcount++;
				}
				radio.releaseRXBuffer();
				MODM_LOG_INFO << "Fetched packet of size 0x" << modm::hex << rxlen
							  << " from Buffer " << modm::ascii << (int)buffer << modm::endl;
			}
			if (!radio.isReceiving())
			{
				if (!radio.startReceive())
				{
					MODM_LOG_ERROR << "Failed to restart receive!" << modm::endl;
				}
			}
			if (buttonPressed)
			{
				buttonPressed = false;
				MODM_LOG_DEBUG << "Sending packet..." << modm::endl;
				radio.stopReceive();
				auto res = radio.transmit(txdata, false, true);
				if (res != MyDw3110::Error::None)
				{
					MODM_LOG_DEBUG << "Failed to transmit!" << modm::endl;
				} else
				{
					txcount++;
				}
			}
		}
	}

	MyDw3110 radio;
	bool packetrdy = false, buttonPressed = false;
	constexpr static size_t RxBufferSize = 1021;  // Maximum supported packet size
	size_t rxlen{0}, rxcount{0}, txcount{0};
	std::array<uint8_t, RxBufferSize> rxdata = {};
	std::array<uint8_t, 12> txdata = {'H', 'e', 'l', 'l', 'o', ' ', 'W', 'o', 'r', 'l', 'd', '!'};
};
RadioThread radioThread;

modm::Fiber fiber_report([] {
	while (true)
	{
		modm::this_fiber::sleep_for(1s);
		radioThread.printStatistics();
	}
});

// Debounce button input
modm::Fiber button_fiber([] {
	modm::ButtonGroup<uint8_t> buttons = modm::ButtonGroup<uint8_t>(0x00);
	const uint8_t ButtonID = 0x01;
	while (true)
	{
		uint8_t buttonState = Button::read() ? ButtonID : 0x00;
		buttons.update(buttonState);
		if (buttons.isPressed(ButtonID)) { radioThread.notifyButton(); }
		modm::this_fiber::sleep_for(10ms);
	}
});

int
main()
{
	Board::initialize();
	LedD13::setOutput();

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
	if (not radioThread.initialize())
	{
		MODM_LOG_ERROR << "Failed to initialize Radio Device!" << modm::endl;
		success = false;
	}
	modm_assert(success, "user", "Failed to initialize devices!");

	MODM_LOG_INFO << "Starting ping pong..." << modm::endl;
	modm::fiber::Scheduler::run();

	return 0;
}
