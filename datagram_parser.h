
#ifndef DATAGRAM_PARSER_H
#define DATAGRAM_PARSER_H

#include <cstdint>

#include "EigenWrapper.h"

typedef uint8_t *buffer_iterator_t;

struct SensorData {
    Vector3 gyro;
    Vector3 accel;

    uint8_t counter;
    uint32_t crc;
};

bool parseDatagram(buffer_iterator_t &it, SensorData &sensor_data);

#endif   // DATAGRAM_PARSER_H
