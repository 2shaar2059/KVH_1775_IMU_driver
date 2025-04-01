
#include "datagram_parser.h"

#include "stim300_constants.h"

using namespace stim300;

// Meta data is stored as "unsigned word", we simply combine the bytes into
// the right sized uint by left shifting them. Note the biggest is the CRC
// which is 32 bits.
static uint32_t parseUnsigned(buffer_iterator_t& it, uint8_t size) {
    uint32_t tmp{0};
    auto* end = it + size;
    while (it < end) {
        tmp = (tmp << 8u) | *(it++);
    }
    return tmp;
}

// Sensor data is stored as two`s complement, we shift the bytes according
// to the datasheet, bu we shift them to fill up the int32_t type then we
// shift them back to their right position. When we shift them back the
// shift operator will automatically sign-extend the value.
static int32_t parseTwosComplement(buffer_iterator_t& it, uint8_t size) {
    int32_t a = (*it) << 24;
    it++;
    int32_t b = (*it) << 16;
    it++;
    if (size == 3) {
        int32_t c = (*it) << 8;
        it++;
        return (a | b | c) >> 8;
    } else {   // size == 2
        return (a | b) >> 16;
    }
}

bool stim300::parseDatagram(buffer_iterator_t& it, SensorData& sensor_data) {
    uint32_t status{0};

    if (IS_INCLUDED[SensorIndx::GYRO]) {
        for (auto& gyro : sensor_data.gyro) {
            gyro =
                GYRO_SCALE * parseTwosComplement(it, N_BYTES_INERTIAL_SENSOR);
        }
        status += parseUnsigned(it, N_BYTES_STATUS);
    }
    if (IS_INCLUDED[SensorIndx::ACC]) {
        for (auto& acc : sensor_data.acc) {
            acc = ACC_SCALE * parseTwosComplement(it, N_BYTES_INERTIAL_SENSOR);
        }
        status += parseUnsigned(it, N_BYTES_STATUS);
    }
    if (IS_INCLUDED[SensorIndx::INCL]) {
        for (auto& incl : sensor_data.incl) {
            incl =
                INCL_SCALE * parseTwosComplement(it, N_BYTES_INERTIAL_SENSOR);
        }
        status += parseUnsigned(it, N_BYTES_STATUS);
    }
    if (IS_INCLUDED[SensorIndx::TEMP]) {
        if (IS_INCLUDED[SensorIndx::GYRO]) {
            for (auto& temp : sensor_data.temp_gyro) {
                temp =
                    TEMP_SCALE * parseTwosComplement(it, N_BYTES_TEMP_SENSOR);
            }
            status += parseUnsigned(it, N_BYTES_STATUS);
        }
        if (IS_INCLUDED[SensorIndx::ACC]) {
            for (auto& temp : sensor_data.temp_acc) {
                temp =
                    TEMP_SCALE * parseTwosComplement(it, N_BYTES_TEMP_SENSOR);
            }
            status += parseUnsigned(it, N_BYTES_STATUS);
        }
        if (IS_INCLUDED[SensorIndx::INCL]) {
            for (auto& temp : sensor_data.temp_incl) {
                temp =
                    TEMP_SCALE * parseTwosComplement(it, N_BYTES_TEMP_SENSOR);
            }
            status += parseUnsigned(it, N_BYTES_STATUS);
        }
    }
    if (IS_INCLUDED[SensorIndx::AUX]) {
        sensor_data.aux =
            AUX_SCALE * parseTwosComplement(it, N_BYTES_AUX_SENSOR);
        status += parseUnsigned(it, N_BYTES_STATUS);
    }
    sensor_data.counter = parseUnsigned(it, N_BYTES_COUNTER);

    sensor_data.latency_us = parseUnsigned(it, N_BYTES_LATENCY);

    sensor_data.crc = parseUnsigned(it, N_BYTES_CRC);

    return status == 0;
}
