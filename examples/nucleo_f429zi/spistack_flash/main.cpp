/*
 * Copyright (c) 2023, Rasmus Kleist Hørlyck Sørensen
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include <modm/board.hpp>

#include <modm/driver/storage/block_device_spiflash.hpp>
#include <modm/driver/storage/block_device_spistack_flash.hpp>

using namespace Board;

void printMemoryContent(const uint8_t* address, std::size_t size) {
	for (std::size_t i = 0; i < size; i++) {
		MODM_LOG_INFO.printf("%x", address[i]);
	}
}

using SpiMaster = SpiMaster1;

// Spi flash chip wiring:
using Cs = GpioA4;
using Mosi = GpioB5;
using Miso = GpioB4;
using Sck = GpioB3;
// Connect WP and HOLD pins to +3V3
// and of course Vdd to +3V3 and Vss to GND


constexpr uint32_t BlockSize = 256;
constexpr uint32_t DieSize = 32*1024*1024;
constexpr uint32_t DieCount = 2;
constexpr uint32_t MemorySize = DieCount * DieSize;
constexpr uint32_t TestMemorySize = 4*1024;
constexpr uint32_t TestMemoryAddress[] = {0, DieSize - TestMemorySize, DieSize, MemorySize - TestMemorySize};

uint8_t bufferA[BlockSize];
uint8_t bufferB[BlockSize];
uint8_t bufferC[BlockSize];

using BdSpiFlash = modm::BdSpiFlash<SpiMaster, Cs, DieSize>;
using BdSpiStackFlash = modm::BdSpiStackFlash<BdSpiFlash, DieCount>;

BdSpiFlash storageDevice;
BdSpiStackFlash storageDeviceStack;

void doMemoryTest()
{
	LedBlue::set();
	MODM_LOG_INFO << "Starting memory test!" << modm::endl;

		for(uint16_t iteration = 0; iteration < 4; iteration++) {

		for(const uint32_t address : TestMemoryAddress) {

			auto dv = std::ldiv(address, DieSize);
			uint8_t* pattern = (iteration % 2 == dv.quot) ? bufferA : bufferB;
			if(!storageDeviceStack.erase(address, TestMemorySize)) {
				MODM_LOG_INFO << "Error: Unable to erase device.";
				return;
			}

			for(uint32_t i = 0; i < TestMemorySize; i += BlockSize) {
				if(!storageDeviceStack.program(pattern, address + i, BlockSize)) {
					MODM_LOG_INFO << "Error: Unable to write data.";
					return;
				}
				MODM_LOG_INFO << ".";
			}
		}

		for(const uint32_t address : TestMemoryAddress) {

			auto dv = std::ldiv(address, DieSize);
			uint8_t* pattern = (iteration % 2 == dv.quot) ? bufferA : bufferB;
			for(uint32_t i = 0; i < TestMemorySize; i += BlockSize) {
				if(!storageDeviceStack.read(bufferC, address + i, BlockSize)) {
					MODM_LOG_INFO << "Error: Unable to read data.";
					return;
				}
				else if(std::memcmp(pattern, bufferC, BlockSize)) {
					MODM_LOG_INFO << "i=" << i << modm::endl;
					MODM_LOG_INFO << "Error: Read '";
					printMemoryContent(bufferC, BlockSize);
					MODM_LOG_INFO << "', expected: '";
					printMemoryContent(pattern, BlockSize);
					MODM_LOG_INFO << "'." << modm::endl;
					return;
				}
			}
		}

		MODM_LOG_INFO << "." << modm::endl;
	}

	MODM_LOG_INFO << modm::endl << "Finished!" << modm::endl;
	LedBlue::reset();
}

int
main()
{
	/**
	 * This example/test writes alternating patterns to a 256 MBit
	 * stacked die flash chip (W25M512VJ) attached to SPI0 using the
	 * `modm::BdSpiStackFlash` block device interface.
	 * The memory content is afterwards read and compared
	 * to the pattern.
	 * Write and read operations are done on 64 byte blocks.
	 *
	 * See above for how to wire the flash chip.
	 */

	// initialize board and SPI
	Board::initialize();
	SpiMaster::connect<Mosi::Mosi, Miso::Miso, Sck::Sck>();
	SpiMaster::initialize<Board::SystemClock, 11_MHz>();

	std::memset(bufferA, 0xAA, BlockSize);
	std::memset(bufferB, 0x55, BlockSize);

	bool initializeSuccess = false;
	if (storageDeviceStack.initialize()) {
		MODM_LOG_INFO << "Erasing complete flash chip... (This may take a while)" << modm::endl;
		if (storageDeviceStack.erase(0, MemorySize)) {
			storageDeviceStack.waitWhileBusy();
			initializeSuccess = true;
		} else {
			MODM_LOG_INFO << "Error: Unable to erase device.";
		}
	} else {
		MODM_LOG_INFO << "Error: Unable to initialize device.";
	}

	if (initializeSuccess) {
		auto id = storageDevice.readId();
		MODM_LOG_INFO << "deviceId=" << id.deviceId << " manufacturerId=" << id.manufacturerId;
		MODM_LOG_INFO << "deviceType=" << id.deviceType << modm::endl;
		MODM_LOG_INFO << "status=" << static_cast<uint8_t>(storageDevice.readStatus()) << modm::endl;
		MODM_LOG_INFO << "Press USER button to start the memory test." << modm::endl;
	}

	while (true)
	{
		if(initializeSuccess && Button::read())
		{
			doMemoryTest();
		}
	}

	return 0;
}
