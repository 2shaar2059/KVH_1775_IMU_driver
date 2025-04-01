
#ifndef DRIVER_STIM300_DATAGRAM_PARSER_H
#define DRIVER_STIM300_DATAGRAM_PARSER_H

#include <array>
#include <cstdint>
#include <vector>

#include "circular_buffer.h"
#include "stim300_constants.h"

typedef uint8_t* buffer_iterator_t;

namespace stim300 {
struct SensorData {
    std::array<double, 3> gyro;
    std::array<double, 3> acc;
    std::array<double, 3> incl;
    std::array<double, 3> temp_gyro;
    std::array<double, 3> temp_acc;
    std::array<double, 3> temp_incl;
    double aux;
    uint8_t counter;
    uint16_t latency_us;
    uint32_t crc;
};

bool parseDatagram(buffer_iterator_t& it, SensorData& sensor_data);

}   // end namespace stim300

#endif   // DRIVER_STIM300_DATAGRAM_PARSER_H
