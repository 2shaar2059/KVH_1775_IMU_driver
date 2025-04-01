
#include <cassert>
#include <cstdint>

#include "stim300_constants.h"
extern "C" {
#include "CRC/crc.h"
}

#include "datagram_parser.h"
#include "driver_stim300.h"
#include "imu_driver_events.h"

DriverStim300::DriverStim300(SerialDriver& serial_driver)
    : serial_driver_(serial_driver),
      checksum_is_ok_(false),
      no_internal_error_(true),
      sensor_data_() {
    // TODO(tushaar): since event services probably wont be ready by the time we
    // try to open the serial port, the event messages sent by open() will not
    // be sent right now (if at all). instead they may appear after the message
    // saying the imu driver app is initilaized. This would communicate an
    // incorrect timeline of events
    serial_driver_.open();
}

Eigen::Vector3d DriverStim300::getAccData() {
    return Eigen::Vector3d(this->sensor_data_.acc[0], this->sensor_data_.acc[1],
                           this->sensor_data_.acc[2]);
}

Eigen::Vector3d DriverStim300::getGyroData() {
    return Eigen::Vector3d(this->sensor_data_.gyro[0],
                           this->sensor_data_.gyro[1],
                           this->sensor_data_.gyro[2]);
}

std::string DriverStim300::getDevicePath() const {
    return serial_driver_.getSerialPortName();
}

void DriverStim300::setDevicePath(const char* newPath) {
    serial_driver_.resetPath(newPath);
}

bool DriverStim300::isChecksumGood() const {
    return checksum_is_ok_;
}

bool DriverStim300::isSensorStatusGood() const {
    return no_internal_error_;
}

void DriverStim300::reset() {
    checksum_is_ok_    = false;
    no_internal_error_ = true;
    serial_driver_.buffer_.reset();
}

void DriverStim300::forceState(bool checksum_is_ok, bool no_internal_error) {
    checksum_is_ok_    = checksum_is_ok;
    no_internal_error_ = no_internal_error;
}

// returns true if a new datagram was found in the byte stream. Whether the
// CRC check passes is another matter
bool DriverStim300::tryReadingNewPacket() {
    int bytesRead = serial_driver_.readBytes();
    if (bytesRead < 1) {
        return false;
    }

    bool foundDatagram = false;
    // try to find a full datagram in the circular buffer
    while (stim300::DATAGRAM_SIZE <= serial_driver_.buffer_.size()) {
        // remove bytes at beginning until the buffer begins with the start byte
        while (serial_driver_.buffer_.get() != stim300::DATAGRAM_ID) {
            serial_driver_.buffer_.remove();
            if (serial_driver_.buffer_.size() < stim300::DATAGRAM_SIZE) {
                return false;
            }
        }
        // found start byte
        foundDatagram = true;

        // copying to an array because later parsing code requires + and <
        // iterator operators, but idk how to define those operators
        // for a circular buffer. Array iterators are easier to work with.
        uint8_t tmp[stim300::DATAGRAM_SIZE];
        int i = 0;
        for (uint8_t byte : serial_driver_.buffer_) {
            if (stim300::DATAGRAM_SIZE <= i) {
                break;
            }
            tmp[i] = byte;
            i++;
        }

        // check status bytes for internal errors and compute checksum
        auto* begin        = tmp;
        auto* it           = tmp + 1;
        no_internal_error_ = stim300::parseDatagram(it, sensor_data_);
        uint32_t checksum  = calculateChecksum(begin, it);
        checksum_is_ok_    = (checksum == sensor_data_.crc);
        if (checksum_is_ok_) {
            // clear the datagram that was just parsed
            for (int j = 0; j < stim300::DATAGRAM_SIZE; j++) {
                serial_driver_.buffer_.remove();
            }
            return true;
        }

        // checksum failed: the datagram was corrupted or the 1st byte
        // happened to match the start byte; remove it and reparse
        serial_driver_.buffer_.remove();
    }

    return foundDatagram;
}

constexpr uint8_t buff_len =
    stim300::DATAGRAM_SIZE - sizeof(uint32_t) + stim300::N_CRC_PADDING_BYTES;
uint32_t calculateChecksum(buffer_iterator_t& begin, buffer_iterator_t& end) {
    uint8_t buffer_CRC[buff_len];

    std::copy(begin, end - sizeof(uint32_t) + stim300::N_CRC_PADDING_BYTES,
              buffer_CRC);

    // Fill the Dummy bytes with 0x00. There are at the end of the buffer
    for (size_t i = 0; i < stim300::N_CRC_PADDING_BYTES; ++i) {
        buffer_CRC[buff_len - (1 + i)] = 0x00;
    }

    crc_t crc = crc_init();

    for (unsigned char& byte : buffer_CRC) {
        crc = crc_update(crc, (unsigned char*)&byte, 1);
    }

    crc = crc_finalize(crc);

    return crc;
}
