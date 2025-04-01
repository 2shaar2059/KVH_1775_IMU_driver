#ifndef DRIVER_STIM300_STIM300_CONSTANTS_H
#define DRIVER_STIM300_STIM300_CONSTANTS_H

#include <termios.h>   // for baudrate id's

#include <array>
#include <cmath>
#include <cstdint>
namespace stim300 {
// All constants taken from:
// https://sensonor.azurewebsites.net/media/5z5lv25o/ts1524-r27-datasheet-stim300.pdf

static constexpr uint8_t N_BYTES_INERTIAL_SENSOR = 3;
static constexpr uint8_t N_BYTES_TEMP_SENSOR     = 2;
static constexpr uint8_t N_BYTES_AUX_SENSOR      = 3;
static constexpr uint8_t N_BYTES_COUNTER         = 1;
static constexpr uint8_t N_BYTES_LATENCY         = 2;
static constexpr uint8_t N_BYTES_CRC             = 4;
static constexpr uint8_t N_BYTES_STATUS          = 1;

// scale factor to use when converting g's reported by IMU to m/s^2
static constexpr double STIM300_GRAVITY = 9.80665;

// The following _SCALE constants are scale factors to convert the (un)signed
// integer parsed from a group of bytes in the datagram to the physical quantity
// measured

// assuming the gyroscope is configured to output Angular Rate (as opposed to
// incremental or integrated angle)
static constexpr double GYRO_SCALE =
    (M_PI / 180.00) / (1u << 14u);   // [rad/sec]

// assuming the accelerometer configured for a 5g range and outputs acceleration
// (as opposed to incremental velocity)
static constexpr double ACC_SCALE =
    (STIM300_GRAVITY) / (1u << 20u);   // [m/sec^2]

// assuming the inclinometer configured to output acceleration (as opposed to
// incremental velocity)
static constexpr double INCL_SCALE =
    (STIM300_GRAVITY) / (1u << 22u);   // [m/sec^2]

static constexpr double TEMP_SCALE = 1 / 256.00;          // [degrees C]
static constexpr double AUX_SCALE  = 5.0 / (1u << 24u);   // [Volts]

//======================================================
// byte that signifies the start of a datagram
constexpr uint8_t DATAGRAM_ID = 0x93;

// FOLLOWING PARAMETERS ASSUME A DATAGRAM ID OF 0x93:

// Number of dummy-bytes to be added for CRC-calculation
constexpr uint8_t N_CRC_PADDING_BYTES = 2;

constexpr uint8_t DATAGRAM_SIZE = 38;

// types of data that coudl be contained in the datagram as configured
enum SensorIndx {
    GYRO = 0,
    ACC,
    INCL,
    TEMP,
    AUX
};

// whether the datagram will hold each type of data in the above enum
constexpr std::array<bool, 5> IS_INCLUDED = {true, true, true, false, false};

// Baudrate
constexpr auto BAUD_ID   = B115200;
constexpr auto BAUD_RATE = 115200;   // [bits/s]

// assume 11 bits/serial_byte (1 start bit + 8 bits/data_byte + 2 stop bits)
constexpr uint32_t DATAGRAM_TRANSIT_TIME_us =
    (uint32_t)((DATAGRAM_SIZE * 11) / (BAUD_RATE / 1e6));   // [us]

}   // namespace stim300

#endif   // DRIVER_STIM300_STIM300_CONSTANTS_H
