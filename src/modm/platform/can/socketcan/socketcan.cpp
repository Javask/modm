/*
 * Copyright (c) 2016, Sascha Schade
 * Copyright (c) 2017, Fabian Greif
 * Copyright (c) 2017, Niklas Hauser
 * Copyright (c) 2023, Christopher Durand
 * Copyright (c) 2024, Michael Jossen
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#include <modm/debug/logger.hpp>

#include "socketcan.hpp"

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <fcntl.h>
#include <unistd.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <string.h>

#undef  MODM_LOG_LEVEL
#define MODM_LOG_LEVEL modm::log::DEBUG

modm::platform::SocketCan::~SocketCan()
{
	close();
}

bool
modm::platform::SocketCan::open(std::string deviceName)
{
	close();

	skt = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if (skt == -1) {
		MODM_LOG_ERROR << MODM_FILE_INFO;
		MODM_LOG_ERROR << "Could not create CAN socket: " << strerror(errno) << modm::endl;
		return false;
	}

	/* Enable FDCAN support */
	int recv_can_fd = 1;
	if (setsockopt(skt, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &recv_can_fd, sizeof(recv_can_fd)) < 0)
	{
		MODM_LOG_ERROR << MODM_FILE_INFO;
		MODM_LOG_ERROR << "Failed to enable FDCAN support: " << strerror(errno) << modm::endl;
		close();
		return false;
	}

	/* Locate the interface you wish to use */
	struct ifreq ifr{};
	if (deviceName.empty() || deviceName.size() > IFNAMSIZ - 1) {
		MODM_LOG_ERROR << MODM_FILE_INFO;
		MODM_LOG_ERROR << "Invalid device name" << modm::endl;
		close();
		return false;
	}
	std::copy(deviceName.begin(), deviceName.end(), ifr.ifr_name);
	ifr.ifr_name[deviceName.size()] = '\0';

	/* ifr.ifr_ifindex gets filled with that device's index */
	if (ioctl(skt, SIOCGIFINDEX, &ifr) == -1) {
		MODM_LOG_ERROR << MODM_FILE_INFO;
		MODM_LOG_ERROR << "Invalid CAN device: " << strerror(errno) << modm::endl;
		close();
		return false;
	}

	/* Select that CAN interface, and bind the socket to it. */
	struct sockaddr_can addr;
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;
	if (bind(skt, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
		MODM_LOG_ERROR << MODM_FILE_INFO;
		MODM_LOG_ERROR << "Could not bind CAN interface: " << strerror(errno) << modm::endl;
		close();
		return false;
	}

	fcntl(skt, F_SETFL, O_NONBLOCK);

	MODM_LOG_DEBUG << MODM_FILE_INFO;
	MODM_LOG_DEBUG << "SocketCAN opened successfully with skt = " << skt << modm::endl;

	return true;
}

void
modm::platform::SocketCan::close()
{
	if (skt != -1) {
		::close(skt);
		skt = -1;
	}
}

modm::Can::BusState
modm::platform::SocketCan::getBusState()
{
	return BusState::Connected;
}

bool
modm::platform::SocketCan::isMessageAvailable()
{
	struct canfd_frame frame;
	int nbytes = recv(skt, &frame, sizeof(struct canfd_frame), MSG_DONTWAIT | MSG_PEEK);

	// recv returns 'Resource temporary not available' which is wired but ignored here.
	/* if (nbytes < 0)
	{
		MODM_LOG_DEBUG << MODM_FILE_INFO;
		MODM_LOG_DEBUG << strerror(errno) << modm::endl;
	} */

	return (nbytes > 0);
}

bool
modm::platform::SocketCan::getMessage(can::Message& message)
{
	struct canfd_frame frame;
	int nbytes = recv(skt, &frame, sizeof(frame), MSG_DONTWAIT);

	if (nbytes > 0)
	{
		if (frame.len > modm::can::Message::capacity)
		{
			MODM_LOG_ERROR << MODM_FILE_INFO;
			MODM_LOG_ERROR << "Received can frame too big for configured buffer." << modm::endl;
			return false;
		}
		message.setIdentifier(frame.can_id & CAN_ERR_MASK);
		message.setLength(frame.len);
		message.flags = {};
		message.setExtended(frame.can_id & CAN_EFF_FLAG);
		if (nbytes == sizeof(struct canfd_frame))
		{
			message.setFlexibleData(true);
			message.flags.brs = (frame.flags & CANFD_BRS);
		}
		message.setRemoteTransmitRequest(frame.can_id & CAN_RTR_FLAG);
		for (uint8_t ii = 0; ii < frame.len; ++ii) {
			message.data[ii] = frame.data[ii];
		}
		return true;
	}
	return false;
}

bool
modm::platform::SocketCan::sendMessage(const can::Message& message)
{
	// Non fd frames do not support message lengths > 8
	if (!message.isFlexibleData() && message.getLength() > 8) { return false; }

	struct canfd_frame frame;

	frame.flags = 0;
	// FDF Frame flag is ignored so we dont need to set it
	if (message.isBitRateSwitching()) { frame.flags |= CANFD_BRS; }

	frame.can_id = message.identifier;
	if (message.isExtended()) {
		frame.can_id |= CAN_EFF_FLAG;
	}
	if (message.isRemoteTransmitRequest()) {
		frame.can_id |= CAN_RTR_FLAG;
	}

	frame.len = message.getLength();

	for (uint8_t ii = 0; ii < message.getLength(); ++ii) {
		frame.data[ii] = message.data[ii];
	}

	// Send can_frame when not explicitly requesting FD, since other applications
	// may not accept canfd_frame. Both structs intentionally share the same
	// layout for this purpose
	const int size = message.isFlexibleData() ? sizeof(canfd_frame) : sizeof(can_frame);
	int bytes_sent = write(skt, &frame, size);

	return (bytes_sent > 0);
}
