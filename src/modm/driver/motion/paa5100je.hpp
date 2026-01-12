/*
 * Copyright (c) 2026, Michael Jossen
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#ifndef MODM_PAA5100JE_HPP
#define MODM_PAA5100JE_HPP

#include <modm/architecture/interface/spi_device.hpp>
#include <modm/math/geometry/vector2.hpp>

namespace modm
{

struct paa5100je
{
	enum class Registers : uint8_t
	{
		ProductID = 0x00,
		RevisionID = 0x01,
		Motion = 0x02,
		DeltaXLow = 0x03,
		DeltaXHigh = 0x04,
		DeltaYLow = 0x05,
		DeltaYHigh = 0x06,
		Squal = 0x07,
		RawDataSum = 0x08,
		RawDataMaximum = 0x09,
		RawDataMinimum = 0x0A,
		ShutterLower = 0x0B,
		ShutterUpper = 0x0C,
		Observation = 0x15,
		MotionBurst = 0x16,
		PowerUpReset = 0x3A,
		Shutdown = 0x3B,
		Resolution = 0x4E,
		RawDataGrab = 0x58,
		RawDataGrabStatus = 0x59,
		Orientation = 0x5B,
		InverseProductID = 0x5F,
		LedEnable = 0x6F,
		WriteProtect = 0x7F,
	};
};

/**
 * Driver for the Paa5100je Motion tracking sensor
 * @author   Michael Jossen
 * @ingroup modm_driver_paa5100je
 */
template<typename SpiMaster, typename Cs>
class Paa5100je : public modm::SpiDevice<SpiMaster>, public paa5100je
{
public:
	using Motion2D = modm::Vector<int16_t, 2>;
	Paa5100je();

	/// @brief Initialize the sensor
	[[nodiscard]]
	bool
	initialize();

	/// @brief Return the product ID of the sensor
	/// Should be 0x49 if the correct sensor is found
	uint8_t
	getProductId();

	/// @brief Return the revision of the sensor
	uint8_t
	getRevision();

	/// @brief Enable or disable LEDs
	void
	setLed(bool enable);

	/// @brief Return the motion data
	/// @param out The data returned by the sensor as a 2D vector
	/// @return true if motion data was present and of good enough quality
	bool
	getMotionData(Motion2D& out);

private:
	constexpr static uint8_t ExpectedProductID = 0x24;

	void
	writeMagic();

	void
	read(Registers reg, uint8_t* data, uint8_t len = 1);

	void
	write(Registers reg, uint8_t val);

	void
	read(uint8_t reg, uint8_t* data, uint8_t len = 1);

	void
	write(uint8_t reg, uint8_t val);

	template<const uint8_t* Val, uint8_t Len>
	void
	writeRaw();

	void
	setChipSelect(bool value);
};

#include "paa5100je_impl.hpp"

}  // namespace modm
#endif