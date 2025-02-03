/*
 * Copyright (c) 2024, Elias H.
 * Copyright (c) 2024, Raphael Lehmann
 * Copyright (c) 2024, Michael Jossen
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef MODM_DW3110_PHY_HPP
#define MODM_DW3110_PHY_HPP

#include <array>
#include <cstdint>
#include <modm/architecture/interface/gpio.hpp>
#include <modm/architecture/interface/spi_device.hpp>
#include <modm/processing/resumable.hpp>
#include <modm/processing/timer/timeout.hpp>
#include <span>

#include "dw3110_definitions.hpp"

namespace modm
{

/**
 * Hardware abstraction layer for DW3110 \n
 * Unsupported Features: AES encryption, Double buffering, GPIO, Temperature and
 * Voltage, Pulse Generator calibration, RX antenna delay temp compensation,
 * Soft reset, Sleep, Sniff mode
 * @ingroup  modm_driver_dw3110
 * @author   Elias H.
 * @author   Raphael Lehmann
 * @author   Michael Jossen
 */
template<typename SpiMaster, typename Cs>
class Dw3110Phy : public modm::SpiDevice<SpiMaster>, protected modm::NestedResumable<6>
{
public:
	enum class TXMode
	{
		Default,
		DefaultAndReceive,
		Force,
		ForceAndReceive,
	};

	enum class DelayTXMode
	{
		AtTime,
		AtTimeAndReceive,
		DelayWRTRX,
		DelayWRTRXAndReceive,
		DelayWRTTX,
		DelayWRTTXAndReceive,
		DelayWRTRef,
		DelayWRTRefAndReceive,
	};

	enum class Error
	{
		None,
		DelayTooShort,
		ChannelBusy,
		TimedOut,
		PayloadTooLarge
	};

	Dw3110Phy();

	/// Set the UWB channel used
	modm::ResumableResult<void>
	setChannel(Dw3110::Channel channel);

	/// Change Header format to non-standard to allow 1021 Byte payloads instead of
	/// default 125
	modm::ResumableResult<void>
	setEnableLongFrames(bool value);

	/// Send the PHR at 6.8Mbps. \n
	/// By default the PHR is always sent at 850kb/s
	modm::ResumableResult<void>
	setSendHeaderFast(bool value);

	/// Set the timeout to wait on arriving packets after startReceive() \n
	/// 0 = No Timeout \n
	/// Register resolution is ~1µs
	modm::ResumableResult<void>
	setReceiveWaitTimeout(modm::chrono::micro_clock::duration duration);

	/// Set whether to stay in receive mode after receive failure
	modm::ResumableResult<void>
	setReenableOnRxFailure(bool value);

	/// Set the time spent listening for competing transmissions on CCA commands \n
	/// Unit is in counts of PAC symbols
	modm::ResumableResult<void>
	setCCATimeout(uint16_t timeout);

	/// Enable a faster TX/RX turnaround. \n
	/// The Frame will be set ready before computation of ToA data has finished and
	/// any ACKs will be sent. \n
	/// Time of Arrival may not be ready when the frame is
	/// made available
	modm::ResumableResult<void>
	setEnableFastTurnaround(bool value);

	/// Read the value of the internal chip clock
	modm::ResumableResult<uint32_t>
	readChipTime();

	/// Set the number of preamble symbols sent with each transmission \n
	/// Each symbol takes ~1µs to send \n
	/// @warning While further fine tuning of preamble length is possible it is currently
	/// unsupported by this driver
	modm::ResumableResult<void>
	setPreambleLength(Dw3110::PreambleLength len);

	/// Get the timestamp of the last arrived packet. \n
	/// This timestamp already has various correction factors applied to it. \n
	/// It is given in ~15.65 picoseconds per unit
	modm::ResumableResult<uint64_t>
	getReceiveTimestamp();

	/// Analogous to getReceiveTimestamp
	modm::ResumableResult<uint64_t>
	getTransmitTimestamp();

	/// Set the time between RX of a packet and the TX of the acknowledgement \n
	/// Specified in number of preamble symbols, so time depends on the PRF
	modm::ResumableResult<void>
	setAcknowledgeTurnaround(uint8_t time);

	/// Set the time between a transmission and the start of RX on any of the RX
	/// after TX commands \n
	/// This can be used to delay turning on of the receiver after transmission
	/// to save on power.
	modm::ResumableResult<void>
	setWaitForResponseTime(modm::PreciseClock::duration time);

	/// Set the start frame delimiter used by the chip
	modm::ResumableResult<void>
	setSFD(Dw3110::StartFrameDelimiter sfd);

	/// Set the preamble code used and looked for.
	/// @warning This implicitly sets the PRF as codes >8 use the 64MHz PRF
	/// instead of 16MHz
	modm::ResumableResult<void>
	setPreambleCode(Dw3110::PreambleCode rx, Dw3110::PreambleCode tx);

	/// Get devices unique id
	modm::ResumableResult<uint64_t>
	getDeviceUID();

	/// Set devices unique id
	modm::ResumableResult<void>
	setDeviceUID(uint64_t uid);

	/// Get devices pan id
	modm::ResumableResult<uint16_t>
	getPanUID();

	/// Get devices short id
	modm::ResumableResult<uint16_t>
	getShortUID();

	/// Set devices pan id
	modm::ResumableResult<void>
	setPanUID(uint16_t pid);

	/// Set devices short id
	modm::ResumableResult<void>
	setShortUID(uint16_t pid);

	/// Enable or disable the frame filtering
	modm::ResumableResult<void>
	setFrameFilterEnabled(bool value);

	/// Get current filtering config
	modm::ResumableResult<Dw3110::FilterConfig_t>
	getFilterConfig();

	/// Set frame filtering config
	modm::ResumableResult<void>
	setFilterConfig(Dw3110::FilterConfig_t fc);

	/// Enable auto acknowledgment \n
	/// Will automatically respond to valid 802.5.14 Frames with the ACK request
	/// bit set \n
	/// Only active when Frame filtering is enabled
	modm::ResumableResult<void>
	setAutoAckEnabled(bool value);

	/// Read the reported state of the chip
	modm::ResumableResult<Dw3110::SystemState>
	getChipState();

	/// Set both the RX and TX antenna delay \n
	/// Unit is approx 15.65ps
	modm::ResumableResult<void>
	setAntennaDelay(uint16_t delay);

	/// Get the currently programmed RX antenna delay \n
	/// Unit is approx 15.65ps
	modm::ResumableResult<uint16_t>
	getRXAntennaDelay();

	/// Get the currently programmed TX antenna delay \n
	/// Unit is approx 15.65ps
	modm::ResumableResult<uint16_t>
	getTXAntennaDelay();

	/// Get the Clock offset multiplier for the received packet \n
	/// Can be used to recover the clock drift compared to a packets sender \n
	/// Unit is in parts per million (ppm)
	modm::ResumableResult<float>
	getReceiverClockOffset();

	/// Runs the RX calibration. \n
	/// Needs to be done after startup, wake and after 20C temperature change \n
	/// Needs to be run when chip is in IDLE_PLL state \n
	/// Automatically run by @ref initialize()
	modm::ResumableResult<bool>
	calibrate();

	/// Initialize the DW3000, make ready to receive/transmit \n
	/// Call ideally after the IRQ Pin has gone high, signalling SPIRDY
	modm::ResumableResult<bool>
	initialize(Dw3110::Channel channel = Dw3110::Channel::Channel5,
			   Dw3110::PreambleCode pcode = Dw3110::PreambleCode::Code_64Mhz_9,
			   Dw3110::PreambleLength plen = Dw3110::PreambleLength::Preamble_64,
			   Dw3110::StartFrameDelimiter sfd = Dw3110::StartFrameDelimiter::IEEE802_15_4z_8);

	/// Set the chip into receive mode
	modm::ResumableResult<bool>
	startReceive();

	/// Check if a packet has been successfully received
	modm::ResumableResult<bool>
	packetReady();

	/// Check if the chip is currently in RX mode
	modm::ResumableResult<bool>
	isReceiving();

	/// Copy received packet into the provided payload buffer, clear packet
	/// received flags
	/// @param payload Region of memory the payload will be written to
	/// @param payload_len Contains the length of the received payload
	modm::ResumableResult<bool>
	fetchPacket(std::span<uint8_t> payload, size_t &payload_len);

	/// Transmit a given package using the current configuration using a given transmission mode
	/// @param payload Span to the desired payload
	/// @param ranging Decides whether or not to set the ranging bit in the header
	/// @param fast Decides if the data portion is sent at 850kbps or 6.8Mbps
	/// @param waitForCompletion If false the function immediately returns after the command is
	/// submitted to the chip
	template<TXMode tmode = TXMode::Default>
	modm::ResumableResult<Error>
	transmit(const std::span<const uint8_t> payload, bool ranging = true, bool fast = true,
			 bool waitForCompletion = true);

	/// Set the reference time value \n
	/// Used as the reference in @ref transmitDelayed() when using DelayTXMode::DelayWRTRef and
	/// DelayTXMode::DelayWRTRefAndReceive
	modm::ResumableResult<void>
	setReferenceTime(uint32_t time);

	/// Transmit a given package using the current configuration using a given delayed transmission
	/// mode
	/// @param payload Span to the desired payload
	/// @param ranging Decides whether or not to set the ranging bit in the header
	/// @param fast Decides if the data portion is sent at 850kbps or 6.8Mbps
	/// @param time Either the transmission time or a delay in units of 4ns dependent on the
	/// DelayTXMode provided
	/// @param waitForCompletion If false the function immediately returns after the command is
	/// submitted to the chip
	/// @warning This transmission mode does not respect the clear channel assessment
	template<DelayTXMode dmode = DelayTXMode::AtTime>
	modm::ResumableResult<Error>
	transmitDelayed(uint32_t time, const std::span<const uint8_t> payload, bool ranging = true,
					bool fast = true, bool waitForCompletion = true);

	/// Read the current system status register
	modm::ResumableResult<Dw3110::SystemStatus_t>
	getStatus();

	/// Clear the given bits in the status register
	modm::ResumableResult<void>
	clearStatusBits(Dw3110::SystemStatus_t mask);

	/// Set interrupt bits \n
	/// Set on which status events the IRQ Pin of the chip should be pulled high \n
	/// Interrupt condition can be cleared by clearing the corresponding status bit using \ref
	/// clearStatusBits() \n
	/// Polarity of the line can be changed with \ref setIRQPolarity() \n
	/// @warning Dw3110::SystemStatus::IRQS will be ignored since it does not correspond to an
	/// interrupt
	modm::ResumableResult<void>
	setInterruptsEnabled(Dw3110::SystemStatus_t mask);

	/// Set the polarity of the interrupt line \n
	/// Default is active high \n
	/// @param high If true sets the line to active high
	modm::ResumableResult<void>
	setIRQPolarity(bool high);

	/// Set the length of the generated Secure timestamp
	///@param len Length in units of 8 chips (~1µs), minimum supported is 32 chips(e.g a value of 3)
	modm::ResumableResult<bool>
	setSTSLength(uint8_t len);

	/// Set the STS mode \n
	/// @param mode Defines where to place the STS inside the packet
	/// @param sdc If true IV and Key are ignored and a deterministic code is used
	modm::ResumableResult<void>
	setSTSMode(Dw3110::STSMode mode, bool sdc);

	/// Return the 12-bit quality assessment of the last received STS
	modm::ResumableResult<uint16_t>
	getSTSQuality();

	/// Return whether or not the STS quality is in an acceptable range
	modm::ResumableResult<bool>
	getSTSGood();

	/// Set the Key to use for AES generation of the STS \n
	/// Ignored if SDC is set in \ref setSTSMode()
	modm::ResumableResult<void>
	setSTSKey(std::span<const uint8_t, 16> key);

	/// Set the IV to use for AES generation of the STS \n
	/// Ignored if SDC is set in \ref setSTSMode()
	modm::ResumableResult<void>
	setSTSIV(std::span<const uint8_t, 16> iv);

	/// Get the Key to use for AES generation of the STS
	modm::ResumableResult<void>
	getSTSKey(std::span<uint8_t, 16> key);

	/// Get the IV to use for AES generation of the STS
	/// @warning Will only return the programmed IV, to get the incremented value use \ref
	/// getCurrentCounter()
	modm::ResumableResult<void>
	getSTSIV(std::span<uint8_t, 16> iv);

	/// Get lower 32 bits of the currently used STS IV
	modm::ResumableResult<uint32_t>
	getCurrentCounter();

	/// Reload the STS IV from the STSIV registers
	modm::ResumableResult<void>
	reloadSTSIV();

	/// Don't increment the STS IV for the next RX/TX
	modm::ResumableResult<void>
	reuseLastSTSIV();

	/// Enable access to CIR information
	modm::ResumableResult<void>
	enableCIRDiagnostics();

	/// Return the index inside the accumulator memory corresponding to the first path in the
	/// channel impulse response estimation
	modm::ResumableResult<uint16_t>
	getFirstPathCIRSampleIndex(Dw3110::TimestampSource ts = Dw3110::TimestampSource::IP);

	/// Return the index inside the accumulator memory corresponding to the peak in the channel
	/// impulse response estimation
	modm::ResumableResult<uint16_t>
	getPeakCIRSampleIndex(Dw3110::TimestampSource ts = Dw3110::TimestampSource::IP);

	/// Return the internally computed 2.12 bit twos-complement fixed point Phase of arrival
	/// estimate either of the STS or the preamble.
	modm::ResumableResult<int16_t>
	getEstimatedPoA(Dw3110::TimestampSource ts = Dw3110::TimestampSource::IP);

	/// Enable access to the raw CIR accumulator memory
	modm::ResumableResult<void>
	enableAccumulatorMemory();

	/// Read a sample from the accumulator memory
	/// Returns a complex value as [low byte real, middle byte real, high byte real, low byte
	/// imaginary, middle byte imaginary, high byte imaginary]
	/// Both the real and imaginary part only have 18 significant bits, the top 6 will always be 0
	modm::ResumableResult<void>
	readAccumulatorMemory(uint16_t index, std::span<uint8_t, 6> out);

protected:
	/// Transmit a given package using the current configuration and a specific command
	/// @param payload Span to the desired payload
	/// @param ranging Decides whether or not to set the ranging bit in the header
	/// @param fast Decides if the data portion is sent at 850kbps or 6.8Mbps
	/// command
	/// @param waitForCompletion If false the function immediately returns after the command is
	/// submitted to the chip
	template<modm::Dw3110::FastCommand Cmd>
	modm::ResumableResult<Error>
	transmitGeneric(std::span<const uint8_t> payload, bool ranging, bool fast, bool waitForCompletion);

	/// Only load configuration independent stuff, everything else should be
	/// initialized when changing those parts
	modm::ResumableResult<void>
	loadOTP();

	/// Check the device ID over SPI
	modm::ResumableResult<bool>
	checkDevID();

	/// Send a command to the chip
	template<Dw3110::FastCommand Cmd>
	modm::ResumableResult<void>
	sendCommand();

	/// Recompute the SFD TOC
	modm::ResumableResult<void>
	setRX_SFD_TOC();

	/// Update the local system_status variable
	modm::ResumableResult<void>
	fetchSystemStatus();

	/// Update the local chip_state variable
	modm::ResumableResult<void>
	fetchChipState();

	/// Due to an errata the check is a bit more complicated \n
	/// Relies on system_status and chip_state being up to date
	modm::ResumableResult<Error>
	checkTXFailed();

	/// Read a variable from the OTP memory
	template<Dw3110::OTPAddr Addr>
	modm::ResumableResult<void>
	readOTPMemory(std::span<uint8_t, 4> out);

	/// Read a variable from a register
	template<Dw3110::Register Reg, size_t Len, size_t Offset = 0>
	modm::ResumableResult<void>
	readRegister(std::span<uint8_t, Len> out);

	/// Read a variable from a register bank
	/// Has to be used for offsets > 127
	template<Dw3110::RegisterBank Reg, size_t Len, size_t Discard>
	modm::ResumableResult<void>
	readRegisterBankIndirect(uint16_t offset, std::span<uint8_t, Len> out);

	/// Read a number of bytes from a register bank, useful for RX buffers and
	/// other large read transfers
	template<Dw3110::RegisterBank Reg>
	modm::ResumableResult<void>
	readRegisterBank(std::span<uint8_t> out, size_t len);

	/// Write a variable to a register
	template<Dw3110::Register Reg, size_t Len, size_t Offset = 0>
	modm::ResumableResult<void>
	writeRegister(std::span<const uint8_t, Len> val);

	/// Simple implementation of read modify write using and and or masks \n
	/// Use on registers that do not support native write register masked
	template<Dw3110::Register Reg, size_t Len, size_t Offset = 0>
	modm::ResumableResult<void>
	readModifyWriteRegister(std::span<const uint8_t, Len> or_mask,
							std::span<const uint8_t, Len> and_mask);

	/// Do not use to clear "write 1 to clear" bits (2.3.1.2 Table 3)
	template<Dw3110::Register Reg, size_t Len, size_t Offset = 0>
	modm::ResumableResult<void>
	writeRegisterMasked(std::span<const uint8_t, Len> or_mask,
						std::span<const uint8_t, Len> and_mask);

	/// Write a number of bytes to a register bank, useful for TX buffers and other
	/// large write transfers
	template<Dw3110::RegisterBank Reg>
	modm::ResumableResult<void>
	writeRegisterBank(std::span<const uint8_t> val, size_t len);

	template<TXMode mode>
	static consteval modm::Dw3110::FastCommand
	txModeToCmd();

	template<DelayTXMode dmode>
	static consteval modm::Dw3110::FastCommand
	txModeToCmdDelay();

private:
	PreciseTimeout timeout;

	Dw3110::SystemStatus_t system_status{0};
	uint16_t preamble_len{0}, sfd_len{0}, pac_len{0}, sfd_toc_val{0}, fcs_len{2};
	bool long_frames{false};
	Dw3110::SystemState chip_state{Dw3110::SystemState::OFF};
	std::array<uint8_t, 16> scratch{};
	std::array<uint8_t, 6> sys_status{}, tx_info{};
	std::array<uint8_t, 2> tx_buffer{}, chan_ctrl{}, rx_sfd_toc{};
	std::array<uint8_t, 4> otp_read{}, rx_cal_res{}, sys_state{}, ldo_config{}, temp_rw{},
		rx_finfo{};
	std::array<uint8_t, 1> xtal{}, bias_ctrl{}, rx_cal_sts{};
	Error last_err;
};

}  // namespace modm

#include "dw3110_phy_impl.hpp"

#endif