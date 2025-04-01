
#ifndef DRIVER_STIM300_DRIVER_STIM300_H
#define DRIVER_STIM300_DRIVER_STIM300_H

extern "C" {
#include "cfe.h"
}
#include "EigenWrapper.h"
#include "datagram_parser.h"
#include "serial_driver.h"

class DriverStim300 {
   public:
    explicit DriverStim300(SerialDriver& serial_driver);

    /** \brief Return the Accelerometers values
     */
    Vector3 getAccData();

    /** \brief Return the Gyroscopes values
     */
    Vector3 getGyroData();

    /** \brief Return the Inclinometers values
     */
    uint16_t getLatency_us() const;
    double getAverageTemp() const;
    std::string getDevicePath() const;

    void setDevicePath(const char* newPath);

    bool isChecksumGood() const;
    bool isSensorStatusGood() const;
    uint8_t getInternalMeasurmentCounter() const;
    bool tryReadingNewPacket();

    // helpful to reset / force state for unit tests
    void reset();
    void forceState(bool checksum_is_ok, bool no_internal_error);

   private:
    SerialDriver& serial_driver_;
    bool checksum_is_ok_;
    bool no_internal_error_;
    stim300::SensorData sensor_data_;
};

uint32_t calculateChecksum(buffer_iterator_t& begin, buffer_iterator_t& end);

#endif   // DRIVER_STIM300_DRIVER_STIM300_H
