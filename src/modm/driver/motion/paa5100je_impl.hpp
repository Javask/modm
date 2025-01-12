/*
 * Copyright (c) 2025, Michael Jossen
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef MODM_PAA5100JE_HPP
#error "Don't include this file directly, use 'paa5100je.hpp' instead!"
#endif

template<typename SpiMaster, typename Cs>
modm::Paa5100je<SpiMaster, Cs>::Paa5100je()
{
	this->attachConfigurationHandler([]() {
		SpiMaster::setDataMode(SpiMaster::DataMode::Mode0);
		SpiMaster::setDataOrder(SpiMaster::DataOrder::MsbFirst);
	});
	Cs::setOutput(true);
}

template<typename SpiMaster, typename Cs>
modm::ResumableResult<bool>
modm::Paa5100je<SpiMaster, Cs>::initialize()
{
	RF_BEGIN();
	wait.restart(50ms);
	RF_WAIT_UNTIL(wait.execute());
	RF_CALL(write(Registers::PowerUpReset, 0x5a));  // 0x5a = Power Up
	wait.restart(20ms);
	RF_WAIT_UNTIL(wait.execute());
	RF_CALL(writeMagic());
	RF_CALL(read(Registers::ProductID, &scratch[0]));
	RF_CALL(read(Registers::InverseProductID, &scratch[1]));
	if (scratch[0] != ExpectedProductID) RF_RETURN(false);
	if (scratch[0] != static_cast<uint8_t>(~scratch[1])) RF_RETURN(false);
	RF_END_RETURN(true);
}

template<typename SpiMaster, typename Cs>
modm::ResumableResult<void>
modm::Paa5100je<SpiMaster, Cs>::setLed(bool enable)
{
	RF_BEGIN();
	RF_CALL(write(Registers::WriteProtect, 0x14));
	RF_CALL(write(Registers::LedEnable, enable ? 0x1c : 0));
	RF_CALL(write(Registers::WriteProtect, 0x00));
	RF_END();
}

template<typename SpiMaster, typename Cs>
modm::ResumableResult<uint8_t>
modm::Paa5100je<SpiMaster, Cs>::getProductId()
{
	RF_BEGIN()
	RF_CALL(read(Registers::ProductID, scratch, 1));
	RF_END_RETURN(scratch[0]);
}

template<typename SpiMaster, typename Cs>
modm::ResumableResult<uint8_t>
modm::Paa5100je<SpiMaster, Cs>::getRevision()
{
	RF_BEGIN()
	RF_CALL(read(Registers::RevisionID, scratch, 1));
	RF_END_RETURN(scratch[0]);
}

template<typename SpiMaster, typename Cs>
modm::ResumableResult<bool>
modm::Paa5100je<SpiMaster, Cs>::getMotionData(Motion2D& out)
{
	RF_BEGIN()
	RF_CALL(read(Registers::MotionBurst, scratch, 12));
	out.x = static_cast<int16_t>(scratch[2]) | static_cast<int16_t>(scratch[3]) << 8;
	out.y = static_cast<int16_t>(scratch[4]) | static_cast<int16_t>(scratch[5]) << 8;
	RF_END_RETURN((scratch[0] & 0x80) != 0 && !(scratch[10] == 0x1F && scratch[6] < 0x19));
}

template<typename SpiMaster, typename Cs>
modm::ResumableResult<void>
modm::Paa5100je<SpiMaster, Cs>::read(Registers reg, uint8_t* data, uint8_t len)
{
	return read(std::to_underlying(reg), data, len);
}

template<typename SpiMaster, typename Cs>
modm::ResumableResult<void>
modm::Paa5100je<SpiMaster, Cs>::read(uint8_t reg, uint8_t* data, uint8_t len)
{
	RF_BEGIN();
	RF_WAIT_UNTIL(this->acquireMaster());

	reg &= 0x7F;  // Clear write bit

	RF_CALL(setChipSelect(false));
	RF_CALL(SpiMaster::transfer(&reg, nullptr, 1));
	RF_CALL(SpiMaster::transfer(nullptr, data, len));
	RF_CALL(setChipSelect(true));

	this->releaseMaster();
	RF_END();
}

template<typename SpiMaster, typename Cs>
modm::ResumableResult<void>
modm::Paa5100je<SpiMaster, Cs>::write(Registers reg, uint8_t val)
{
	return write(std::to_underlying(reg), val);
}

template<typename SpiMaster, typename Cs>
modm::ResumableResult<void>
modm::Paa5100je<SpiMaster, Cs>::write(uint8_t reg, uint8_t val)
{
	RF_BEGIN();
	RF_WAIT_UNTIL(this->acquireMaster());

	reg |= 0x80;  // Set write bit

	RF_CALL(setChipSelect(false));
	RF_CALL(SpiMaster::transfer(&reg, nullptr, 1));
	RF_CALL(SpiMaster::transfer(&val, nullptr, 1));
	RF_CALL(setChipSelect(true));

	this->releaseMaster();
	RF_END();
}

template<typename SpiMaster, typename Cs>
template<const uint8_t* Val, uint8_t Len>
modm::ResumableResult<void>
modm::Paa5100je<SpiMaster, Cs>::writeRaw()
{
	static_assert(Len % 2 == 0);
	RF_BEGIN();

	RF_WAIT_UNTIL(this->acquireMaster());

	for (uint16_t i = 0; i < Len; i += 2)
	{
		RF_CALL(setChipSelect(false));
		RF_CALL(SpiMaster::transfer(&Val[i], nullptr, 2));
		RF_CALL(setChipSelect(true));
	}

	this->releaseMaster();
	RF_END();
}

template<typename SpiMaster, typename Cs>
modm::ResumableResult<void>
modm::Paa5100je<SpiMaster, Cs>::writeMagic()
{
	// Lots of magic apparently needed for the sensor to function correctly, explained nowhere
	RF_BEGIN();
	constexpr static uint8_t magic1[] = {
		0xFF, 0x00, 0xD5, 0x01, 0xD0, 0x07, 0xFF, 0x0E, 0xC3, 0x10,
	};
	RF_CALL(writeRaw<magic1, sizeof(magic1)>());
	RF_CALL(read(0x67, &scratch[0]));
	scratch[0] = (scratch[0] & 0x80) ? 0x04 : 0x02;
	RF_CALL(write(0x48, scratch[0]));
	constexpr static uint8_t magic2[] = {
		0xFF, 0x00, 0xD1, 0x7B, 0xD0, 0x00, 0xD5, 0x00, 0xFF, 0x0E,
	};
	RF_CALL(writeRaw<magic2, sizeof(magic2)>());
	RF_CALL(read(0x73, &scratch[0]));
	if (scratch[0] == 0x00)
	{
		RF_CALL(read(0x70, &scratch[0]));
		RF_CALL(read(0x71, &scratch[1]));
		if (scratch[0] <= 28) { scratch[0] += 14; }
		if (scratch[0] > 28) { scratch[0] += 11; }
		if (scratch[0] > 0x3F) { scratch[0] = 0x3f; }
		scratch[1] = (scratch[1] * 45) / 100;
		constexpr static uint8_t magic3[] = {
			0xFF, 0x00, 0xE1, 0xAD, 0xD1, 0x70, 0xFF, 0x0E,
		};
		RF_CALL(writeRaw<magic3, sizeof(magic3)>());
		RF_CALL(write(0x70, scratch[0]));
		RF_CALL(write(0x71, scratch[1]));
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
	RF_CALL(writeRaw<magic4, sizeof(magic4)>());
	wait.restart(10ms);
	RF_WAIT_UNTIL(wait.execute());
	constexpr static uint8_t magic5[] = {
		0xFF, 0x00, 0xB2, 0x00, 0xFF, 0x07, 0xC0, 0x40, 0xFF, 0x06, 0xE8,
		0xF0, 0xE9, 0x00, 0xFF, 0x0D, 0xC8, 0xC0, 0xEF, 0xD5, 0xFF, 0x00,
		0xDB, 0xA0, 0xCE, 0xA8, 0xDA, 0x90, 0xC0, 0x80, 0xF3, 0x1F,
	};
	RF_CALL(writeRaw<magic5, sizeof(magic5)>());
	wait.restart(240ms);
	RF_WAIT_UNTIL(wait.execute());
	constexpr static uint8_t magic6[] = {0xF3, 0x00};
	RF_CALL(writeRaw<magic6, sizeof(magic6)>());
	RF_END();
}

template<typename SpiMaster, typename Cs>
modm::ResumableResult<void>
modm::Paa5100je<SpiMaster, Cs>::setChipSelect(bool value)
{
	RF_BEGIN();
	if (!value)
	{
		Cs::setOutput(false);
	} else
	{
		cs_timeout.restart(1us);
		RF_WAIT_UNTIL(cs_timeout.execute());
		Cs::setOutput(true);
		cs_timeout.restart(1us);
		RF_WAIT_UNTIL(cs_timeout.execute());
	}
	RF_END();
}