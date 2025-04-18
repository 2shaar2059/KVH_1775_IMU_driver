
#include "datagram_parser.h"

#include <cstdint>

#include "constants.h"
using namespace std;

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

// SPFP stand for "Single Precision Floating Point". Just reinterpreting 32-bit
// long bit-pattern as single precision IEEE 754 float
static float parseSPFP(buffer_iterator_t& it) {
    uint32_t a = (*it) << 24u;
    it++;
    uint32_t b = (*it) << 16u;
    it++;
    uint32_t c = (*it) << 8u;
    it++;
    uint32_t d = (*it);
    it++;
    uint32_t bytes = (a | b | c | d);
    return *((float*)&bytes);
}

bool parseDatagram(buffer_iterator_t& it, SensorData& sensor_data) {
    sensor_data.gyro.x = parseSPFP(it);
    sensor_data.gyro.y = parseSPFP(it);
    sensor_data.gyro.z = parseSPFP(it);

    sensor_data.accel.x = ACC_SCALE * parseSPFP(it);
    sensor_data.accel.y = ACC_SCALE * parseSPFP(it);
    sensor_data.accel.z = ACC_SCALE * parseSPFP(it);

    // Temperature and magnetic data??
    (void)parseSPFP(it);

    uint8_t status = parseUnsigned(it, N_BYTES_STATUS);
    // cout << status << "\t";
    sensor_data.counter = parseUnsigned(it, N_BYTES_COUNTER);
    // cout << (int)sensor_data.counter << "\t";
    sensor_data.crc = parseUnsigned(it, N_BYTES_CRC);
    // cout << sensor_data.crc << "\t";
    // cout << endl;

    const uint8_t nominal_status = 0b0111'0111;
    return status == nominal_status;
}
