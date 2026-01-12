/*
 * Copyright (c) 2026, Michael Jossen
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * This file incorporates work covered by the following copyright and
 * permission notice:
 *   MIT License
 *
 *   Copyright (c) 2018 Pimoroni Ltd.
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:
 *
 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.
 *
 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */
// ----------------------------------------------------------------------------
// Adapted from https://github.com/pimoroni/pmw3901-python

#ifndef MODM_PAA5100JE_HPP
#error "Don't include this file directly, use 'paa5100je.hpp' instead!"
#endif
#include <modm/processing/fiber.hpp>

template<typename SpiMaster, typename Cs>
Paa5100je<SpiMaster, Cs>::Paa5100je()
{
	this->attachConfigurationHandler([]() {
		SpiMaster::setDataMode(SpiMaster::DataMode::Mode0);
		SpiMaster::setDataOrder(SpiMaster::DataOrder::MsbFirst);
	});
	Cs::setOutput(true);
}

template<typename SpiMaster, typename Cs>
bool
Paa5100je<SpiMaster, Cs>::initialize()
{

	modm::this_fiber::sleep_for(50ms);
	write(Registers::PowerUpReset, 0x5a);  // 0x5a = Power Up
	modm::this_fiber::sleep_for(20ms);
	writeMagic();
	uint8_t id, inv_id;
	read(Registers::ProductID, &id);
	read(Registers::InverseProductID, &inv_id);
	if (id != ExpectedProductID) return false;
	if (id != static_cast<uint8_t>(~inv_id)) return false;
	return true;
}

template<typename SpiMaster, typename Cs>
void
Paa5100je<SpiMaster, Cs>::setLed(bool enable)
{

	write(Registers::WriteProtect, 0x14);
	write(Registers::LedEnable, enable ? 0x1c : 0);
	write(Registers::WriteProtect, 0x00);
}

template<typename SpiMaster, typename Cs>
uint8_t
Paa5100je<SpiMaster, Cs>::getProductId()
{
	uint8_t id;
	read(Registers::ProductID, &id, 1);
	return id;
}

template<typename SpiMaster, typename Cs>
uint8_t
Paa5100je<SpiMaster, Cs>::getRevision()
{
	uint8_t rev;
	read(Registers::RevisionID, &rev, 1);
	return rev;
}

template<typename SpiMaster, typename Cs>
bool
Paa5100je<SpiMaster, Cs>::getMotionData(Motion2D& out)
{
	uint8_t data[12];
	read(Registers::MotionBurst, data, 12);
	out.x = static_cast<int16_t>(data[2]) | static_cast<int16_t>(data[3]) << 8;
	out.y = static_cast<int16_t>(data[4]) | static_cast<int16_t>(data[5]) << 8;
	return (data[0] & 0x80) != 0 && !(data[10] == 0x1F && data[6] < 0x19);
}

template<typename SpiMaster, typename Cs>
void
Paa5100je<SpiMaster, Cs>::read(Registers reg, uint8_t* data, uint8_t len)
{
	return read(std::to_underlying(reg), data, len);
}

template<typename SpiMaster, typename Cs>
void
Paa5100je<SpiMaster, Cs>::read(uint8_t reg, uint8_t* data, uint8_t len)
{
	while (!this->acquireMaster()) { modm::this_fiber::yield(); }

	reg &= 0x7F;  // Clear write bit

	setChipSelect(false);
	SpiMaster::transfer(&reg, nullptr, 1);
	SpiMaster::transfer(nullptr, data, len);
	setChipSelect(true);

	this->releaseMaster();
}

template<typename SpiMaster, typename Cs>
void
Paa5100je<SpiMaster, Cs>::write(Registers reg, uint8_t val)
{
	return write(std::to_underlying(reg), val);
}

template<typename SpiMaster, typename Cs>
void
Paa5100je<SpiMaster, Cs>::write(uint8_t reg, uint8_t val)
{

	while (!this->acquireMaster()) { modm::this_fiber::yield(); }

	reg |= 0x80;  // Set write bit

	setChipSelect(false);
	SpiMaster::transfer(&reg, nullptr, 1);
	SpiMaster::transfer(&val, nullptr, 1);
	setChipSelect(true);

	this->releaseMaster();
}

template<typename SpiMaster, typename Cs>
template<const uint8_t* Val, uint8_t Len>
void
Paa5100je<SpiMaster, Cs>::writeRaw()
{
	static_assert(Len % 2 == 0);

	while (!this->acquireMaster()) { modm::this_fiber::yield(); }

	for (uint16_t i = 0; i < Len; i += 2)
	{
		setChipSelect(false);
		SpiMaster::transfer(&Val[i], nullptr, 2);
		setChipSelect(true);
	}

	this->releaseMaster();
}

template<typename SpiMaster, typename Cs>
void
Paa5100je<SpiMaster, Cs>::setChipSelect(bool value)
{

	if (!value)
	{
		Cs::setOutput(false);
	} else
	{
		modm::this_fiber::sleep_for(1us);
		Cs::setOutput(true);
		modm::this_fiber::sleep_for(1us);
	}
}

template<typename SpiMaster, typename Cs>
void
Paa5100je<SpiMaster, Cs>::writeMagic()
{
	// Lots of magic apparently needed for the sensor to function correctly, explained nowhere
	uint8_t scratch[2];
	constexpr static uint8_t magic1[] = {
		0xFF, 0x00, 0xD5, 0x01, 0xD0, 0x07, 0xFF, 0x0E, 0xC3, 0x10,
	};
	writeRaw<magic1, sizeof(magic1)>();
	read(0x67, &scratch[0]);
	scratch[0] = (scratch[0] & 0x80) ? 0x04 : 0x02;
	write(0x48, scratch[0]);
	constexpr static uint8_t magic2[] = {
		0xFF, 0x00, 0xD1, 0x7B, 0xD0, 0x00, 0xD5, 0x00, 0xFF, 0x0E,
	};
	writeRaw<magic2, sizeof(magic2)>();
	read(0x73, &scratch[0]);
	if (scratch[0] == 0x00)
	{
		read(0x70, &scratch[0]);
		read(0x71, &scratch[1]);
		if (scratch[0] <= 28) { scratch[0] += 14; }
		if (scratch[0] > 28) { scratch[0] += 11; }
		if (scratch[0] > 0x3F) { scratch[0] = 0x3f; }
		scratch[1] = (scratch[1] * 45) / 100;
		constexpr static uint8_t magic3[] = {
			0xFF, 0x00, 0xE1, 0xAD, 0xD1, 0x70, 0xFF, 0x0E,
		};
		writeRaw<magic3, sizeof(magic3)>();
		write(0x70, scratch[0]);
		write(0x71, scratch[1]);
	}
	constexpr static uint8_t magic4[] = {
		0xFF, 0x00, 0xE1, 0xAD, 0xFF, 0x03, 0xC0, 0x00, 0xFF, 0x05, 0xC1, 0xB3, 0xC3, 0xF1, 0xC5,
		0x14, 0xDF, 0x34, 0xFB, 0x08, 0xDE, 0x34, 0xDB, 0x11, 0xED, 0x11, 0xC5, 0x17, 0xF0, 0xE5,
		0xF1, 0xE5, 0xFF, 0x06, 0xC4, 0x1B, 0xC0, 0xBF, 0xCE, 0x3F, 0xFF, 0x08, 0xE6, 0x44, 0xE5,
		0x20, 0xEA, 0x3A, 0xE1, 0x05, 0xE2, 0x05, 0xFF, 0x09, 0xCF, 0xAF, 0xDF, 0x40, 0xC8, 0x80,
		0xC9, 0x80, 0xD7, 0x77, 0xE0, 0x78, 0xE1, 0x78, 0xE2, 0x08, 0xE3, 0x50, 0xFF, 0x0A, 0xC5,
		0x60, 0xFF, 0x00, 0xCD, 0x11, 0xD5, 0x80, 0xF4, 0x21, 0xF5, 0x1F, 0xCA, 0x78, 0xCB, 0x78,
		0xC4, 0x08, 0xC5, 0x50, 0xE4, 0xFF, 0xE5, 0x1F, 0xFF, 0x14, 0xE5, 0x67, 0xE6, 0x08, 0xE3,
		0x70, 0xEF, 0x1C, 0xFF, 0x15, 0xC8, 0x48, 0xFF, 0x07, 0xC1, 0x0D, 0xC3, 0x14, 0xCB, 0x0E,
		0xC5, 0x0F, 0xC4, 0x42, 0xCC, 0x80, 0xFF, 0x10, 0xDB, 0x02, 0xFF, 0x07, 0xC0, 0x41,
	};
	writeRaw<magic4, sizeof(magic4)>();
	modm::this_fiber::sleep_for(10ms);
	constexpr static uint8_t magic5[] = {
		0xFF, 0x00, 0xB2, 0x00, 0xFF, 0x07, 0xC0, 0x40, 0xFF, 0x06, 0xE8,
		0xF0, 0xE9, 0x00, 0xFF, 0x0D, 0xC8, 0xC0, 0xEF, 0xD5, 0xFF, 0x00,
		0xDB, 0xA0, 0xCE, 0xA8, 0xDA, 0x90, 0xC0, 0x80, 0xF3, 0x1F,
	};
	writeRaw<magic5, sizeof(magic5)>();
	modm::this_fiber::sleep_for(240ms);
	constexpr static uint8_t magic6[] = {0xF3, 0x00};
	writeRaw<magic6, sizeof(magic6)>();
}
