/*
 * Copyright (c) 2013-2017, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#ifndef MODM_SOFTWARE_GPIO_HPP
#define MODM_SOFTWARE_GPIO_HPP

#include <modm/architecture/utils.hpp>
#include <modm/architecture/interface/gpio.hpp>
#include <modm/utils/template_metaprogramming.hpp>

namespace modm
{

namespace platform
{

/**
 * Dummy implementation of an I/O pin.
 *
 * This class can be used when a pin is not required. All functions
 * are dummy functions which do nothing. `read()` will always
 * return `false`.
 *
 * For example when creating a software SPI with the modm::SoftwareSimpleSpi
 * class and the return channel (MISO - Master In Slave Out) is not needed,
 * a good way is to use this class as a parameter when defining the
 * SPI class.
 *
 * Example:
 * @code
 * #include <modm/architecture/platform.hpp>
 *
 * namespace pin
 * {
 *     typedef GpioOutputD7 Clk;
 *     typedef GpioOutputD5 Mosi;
 * }
 *
 * BitBangSpiMaster< pin::Clk, pin::Mosi, GpioUnused > Spi;
 *
 * ...
 * Spi::write(0xaa);
 * @endcode
 *
 * @author	Fabian Greif
 * @ingroup	gpio
 */
class GpioUnused : GpioIO
{
public:
	modm_always_inline static void
	setOutput()
	{
	}

	modm_always_inline static void
	setOutput(bool)
	{
	}

	modm_always_inline static void
	setInput()
	{
	}

	modm_always_inline static void
	set()
	{
	}

	modm_always_inline static void
	set(bool)
	{
	}

	modm_always_inline static void
	reset()
	{
	}

	modm_always_inline static void
	toggle()
	{
	}

	/// Always returns `false`
	modm_always_inline static bool
	read()
	{
		return false;
	}
};

/**
 * Invert a pin in software.
 *
 * This template can be used the invert the logic level of a normal
 * pin template.
 *
 * Example:
 * @code
 * #include <modm/architecture/platform.hpp>
 *
 * typedef GpioInverted< GpioOutputB0 > Led;
 *
 * ...
 * Led::setOutput();
 * Led::reset();
 * @endcode
 *
 * @author	Fabian Greif
 * @ingroup	gpio
 */
template < class Pin >
class GpioInverted : public Pin
{
public:
	modm_always_inline static void
	setOutput()
	{
		Pin::setOutput();
	}

	modm_always_inline static void
	setOutput(bool value)
	{
		Pin::setOutput(!value);
	}

	modm_always_inline static void
	set()
	{
		Pin::reset();
	}

	modm_always_inline static void
	set(bool value)
	{
		Pin::set(!value);
	}

	modm_always_inline static void
	reset()
	{
		Pin::set();
	}

	modm_always_inline static bool
	read()
	{
		return !Pin::read();
	}
};


/**
 * Create an up to 16-bit port from arbitrary pins.
 *
 * Be aware that this method is slow, because for every write or read
 * cycle, every of the up to sixteen pins have to be read/written individually.
 *
 * This class is included here because it can be very useful sometimes,
 * for example when connecting a LCD where speed is not the main concern.
 *
 * Whenever possible prefer the native `GpioPort` implementations
 * over this class as they are much faster and require less code.
 *
 * This class optimizes the data type for the `read()` and `write()` methods.
 * Supplying up to 8 Gpios will use `uint8_t`, up to 16 Gpios will use
 * `uint16_t`.
 *
 * @note Since the bit order is explicitly given by the order of template arguments,
 *       this class only supports `DataOrder::Normal`.
 *       If you need reverse bit order, reverse the order of `Gpios`!
 *
 * @tparam Gpios	Up to 16 GpioIO classes, ordered MSB to LSB
 *
 * @author	Niklas Hauser
 * @ingroup	gpio
 */
template<typename... Gpios>
class SoftwareGpioPort : public ::modm::GpioPort
{
#ifdef __DOXYGEN__
public:
	static constexpr uint8_t width = sizeof(Gpios);
	static constexpr DataOrder getDataOrder();

	using PortType = uint16_t;

	static void
	setOutput();

	static void
	setInput();

	static PortType
	read();

	static void
	write(PortType data);

	static void
	toggle();
#endif
};

/// @cond
template<typename Gpio, typename... Gpios>
class SoftwareGpioPort<Gpio, Gpios...> : private SoftwareGpioPort<Gpios...>
{
public:
	static constexpr uint8_t width = 1 + SoftwareGpioPort<Gpios...>::width;

	static constexpr ::modm::GpioPort::DataOrder
	getDataOrder()
	{ return ::modm::GpioPort::DataOrder::Normal; }
private:
	typedef typename modm::tmp::Select< (width > 8),
										uint16_t,
										uint8_t >::Result PortType;

public:
	static void
	setOutput()
	{
		Gpio::setOutput();
		SoftwareGpioPort<Gpios...>::setOutput();
	}

	static void
	setInput()
	{
		Gpio::setInput();
		SoftwareGpioPort<Gpios...>::setInput();
	}

	static PortType
	read()
	{
		PortType data = SoftwareGpioPort<Gpios...>::read();
		if (Gpio::read()) {
			data |= (1 << (width-1));
		}
		return data;
	}

	static void
	write(PortType data)
	{
		Gpio::set(data & (1 << (width-1)));
		SoftwareGpioPort<Gpios...>::write(data);
	}

	static void
	toggle()
	{
		Gpio::toggle();
		SoftwareGpioPort<Gpios...>::toggle();
	}
};

template<>
class SoftwareGpioPort<>
{
public:
	static constexpr uint8_t width = 0;

	static void
	setOutput() {}

	static void
	setInput() {}

	static uint8_t
	read() { return 0; }

	static void
	write(uint8_t /*data*/) {}

	static void
	toggle() {}
};
/// @endcond

}	// namespace platform

}	// namespace modm

#endif // MODM_SOFTWARE_GPIO_HPP
