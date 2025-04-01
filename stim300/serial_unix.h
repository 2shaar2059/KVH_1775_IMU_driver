#ifndef DRIVER_STIM300_SERIAL_UBUNTU_H
#define DRIVER_STIM300_SERIAL_UBUNTU_H

extern "C" {
#include "cfe.h"
#include "cfe_evs.h"
}

#include <termios.h>

#include <string>

#include "serial_driver.h"

/* This class is for actually interfacing with the IMU (connecting with it,
 * reading data from it, reconnecting, closing the connection, etc).
 */
class SerialUnix : public SerialDriver {
   public:
    virtual ~SerialUnix() = default;
    SerialUnix()          = default;

    /**
     * @param: serial_port_name: device filepath (e.g. "/dev/ttyUSB0")
     */
    explicit SerialUnix(const std::string& serial_port_name);
    void open() override;
    ssize_t readBytes() override;
    void reconnect() override;

   private:
    struct termios config_ {};
};

#endif   // DRIVER_STIM300_SERIAL_UBUNTU_H
