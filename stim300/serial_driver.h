
#ifndef DRIVER_STIM300_SERIAL_DRIVER_H
#define DRIVER_STIM300_SERIAL_DRIVER_H

#include <string>

#include "circular_buffer.h"
#include "stim300_constants.h"

#define buffSize (2 * stim300::DATAGRAM_SIZE)

class SerialDriver {
   public:
    SerialDriver() = default;
    explicit SerialDriver(const std::string& serial_port_name);
    virtual void open()         = 0;
    virtual ssize_t readBytes() = 0;
    virtual void reconnect()    = 0;
    virtual ~SerialDriver();

    void resetPath(const std::string& new_serial_port_name);
    bool isPortOpened();
    const std::string& getSerialPortName() const;
    CircularBuffer<uint8_t, buffSize> buffer_;

   protected:
    std::string serial_port_name_;
    bool openedPort_ = false;
    int file_handle_{};

    bool firstReadError = true;
    bool firstPollError = true;

    OS_time_t timeOfLastReadErrMsgSent{0, 0};
    OS_time_t timeOfLastPollErrMsgSent{0, 0};
};

#endif   // DRIVER_STIM300_SERIAL_DRIVER_H
