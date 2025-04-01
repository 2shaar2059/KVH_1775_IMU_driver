
extern "C" {
#include "cfe.h"
#include "cfe_evs.h"
}

#include <fcntl.h>
#include <poll.h>
#include <unistd.h>

#include <cstring>   // for strerror_l()

#include "common_utils.h"
#include "imu_driver_events.h"
#include "moonranger_output_strings.h"
#include "serial_driver.h"
#include "serial_unix.h"
#include "stim300_constants.h"

// Everything is learned from
// https://en.wikibooks.org/wiki/Serial_Programming/termios
// "termios is the newer (now already a few decades old)
// Unix API for terminal I/O"

SerialUnix::SerialUnix(const std::string& serial_port_name)
    : SerialDriver(serial_port_name) {
}

void SerialUnix::open() {
    /*O_RDONLY: Opens the port for reading only
    O_NOCTTY: The port never becomes the controlling terminal of the process
    O_NDELAY: Use non-blocking I/O. On some systems this also means the RS232
    DCD signal line is ignored*/
    file_handle_   = ::open(serial_port_name_.c_str(),
                            O_RDONLY | O_NOCTTY | O_NDELAY | O_CLOEXEC);
    int savedErrno = errno;
    if (file_handle_ == -1) {
        CFE_EVS_SendEvent(IMU_DRIVER_SYSCALL_OPEN_ERROR,
                          CFE_EVS_EventType_CRITICAL,
                          MOON_ERROR_INFO("Failed to open IMU at %s: %s"),
                          serial_port_name_.c_str(), errnoToString(savedErrno));
        openedPort_ = false;
        return;
    }

    // Check if the file descriptor is pointing to a TTY device or not.
    int retval = isatty(file_handle_);
    savedErrno = errno;
    if (retval != 1) {
        CFE_EVS_SendEvent(IMU_DRIVER_SYSCALL_ISTTY_ERROR,
                          CFE_EVS_EventType_CRITICAL,
                          MOON_ERROR_INFO("Serial port is not TTY device: %s"),
                          errnoToString(savedErrno));
        return;
    }

    // Get the current configuration of the serial interface
    retval     = tcgetattr(file_handle_, &config_);
    savedErrno = errno;
    if (retval != 0) {
        CFE_EVS_SendEvent(
            IMU_DRIVER_SYSCALL_TCGETATTR_ERROR, CFE_EVS_EventType_CRITICAL,
            MOON_ERROR_INFO("Couldn't retrieve current serial config: %s"),
            errnoToString(savedErrno));
        return;
    }

    //
    // Input flags - Turn off input processing
    //
    // convert break to null byte, no CR to NL translation,
    // no NL to CR translation, don't mark parity errors or breaks
    // no input parity check, don't strip high bit off,
    // no XON/XOFF software flow control
    //
    config_.c_iflag &=
        ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);

    //
    // Output flags - Turn off output processing
    //
    // no CR to NL translation, no NL to CR-NL translation,
    // no NL to CR translation, no column 0 CR suppression,
    // no Ctrl-D suppression, no fill characters, no case mapping,
    // no local output processing
    //
    // config_.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
    //                     ONOCR | ONOEOT| OFILL | OLCUC | OPOST);
    config_.c_oflag = 0;

    //
    // No line processing
    //
    // echo off, echo newline off, canonical mode off,
    // extended input processing off, signal chars off
    config_.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

    //
    // Turn off character processing
    //
    // clear current char size mask, no parity checking,
    // no output processing, force 8 bit input
    config_.c_cflag &= ~(CSIZE | PARENB);
    config_.c_cflag |= CS8;
    config_.c_cflag |= CSTOPB;   // 2 stop bits

    config_.c_cc[VMIN]  = 1;   // One input byte is enough to return from read()
    config_.c_cc[VTIME] = 0;   // Inter-character timer off

    // Communication speed
    retval = cfsetispeed(&config_, stim300::BAUD_ID);
    savedErrno = errno;
    if (retval != 0) {
        CFE_EVS_SendEvent(
            IMU_DRIVER_SYSCALL_CFSETIOSPEED_ERROR, CFE_EVS_EventType_CRITICAL,
            MOON_ERROR_INFO("Input baud rate couldn't be set: %s"),
            errnoToString(savedErrno));
        return;
    }
    retval = cfsetospeed(&config_, stim300::BAUD_ID);
    savedErrno = errno;
    if (retval != 0) {
        CFE_EVS_SendEvent(
            IMU_DRIVER_SYSCALL_CFSETIOSPEED_ERROR, CFE_EVS_EventType_CRITICAL,
            MOON_ERROR_INFO("Output baud rate couldn't be set: %s"),
            errnoToString(savedErrno));
        return;
    }

    // Finally, apply the configuration
    retval     = tcsetattr(file_handle_, TCSAFLUSH, &config_);
    savedErrno = errno;
    if (retval != 0) {
        CFE_EVS_SendEvent(
            IMU_DRIVER_SYSCALL_TCSETATTR_ERROR, CFE_EVS_EventType_CRITICAL,
            MOON_ERROR_INFO("Couldn't apply any serial settings: %s"),
            errnoToString(savedErrno));

        return;
    }

    CFE_EVS_SendEvent(IMU_DRIVER_SYSCALL_OPEN_SUCCESS,
                      CFE_EVS_EventType_INFORMATION,
                      MOON_SUCCESS_INFO("IMU connected successfully!"));
    openedPort_ = true;

    /*TODO: do we need to set exclusive access to the serial device?
        Code to accomplish setting exclusive access:
       https://en.wikibooks.org/wiki/Serial_Programming/termios#Exclusive_Access
        */
}

void SerialUnix::reconnect() {
    if (0 <= file_handle_) {
        // only close the fd if it's nonnegative (i.e. open() returned a valid
        // fd and we arent just trying to reconnect because open failed
        // previously)
        int closeResult = ::close(file_handle_);
        int savedErrno  = errno;
        if (-1 == closeResult) {
            CFE_EVS_SendEvent(IMU_DRIVER_SYSCALL_CLOSE_ERROR,
                              CFE_EVS_EventType_CRITICAL,
                              "IMU_DRIVER: Failed to close fd %d: %s",
                              file_handle_, errnoToString(savedErrno));
        }
    }
    open();
}

ssize_t SerialUnix::readBytes() {
    OS_time_t currTime;
    CFE_PSP_GetTime(&currTime);

    // poll to check if file descriptor is ready for reading
    struct pollfd pfd {};
    pfd.fd     = file_handle_;
    pfd.events = POLLIN;

    int ms_timeout = 1000;
    int pollResult = poll(&pfd, 1, ms_timeout);
    int savedErrno = errno;
    if (pollResult < 1) {   // error
        if (firstPollError or
            elapsedTime(timeOfLastPollErrMsgSent, currTime) > 5) {
            CFE_EVS_SendEvent(
                IMU_DRIVER_SYSCALL_POLL_ERROR, CFE_EVS_EventType_CRITICAL,
                MOON_ERROR_INFO("IMU Driver Serial Port poll() error: "
                                "%s. Attempting reconnect."),
                errnoToString(savedErrno));
            reconnect();
            timeOfLastPollErrMsgSent = currTime;
        }
        firstPollError = false;
        return pollResult;
    }

    // poll() will return as soon as even 1 byte arrives from the IMU.
    // But, since it will take some time for the rest of the
    // datagram to arrive, we should wait a little bit before reading
    // the serial port
    usleep(stim300::DATAGRAM_TRANSIT_TIME_us);

    // actually performing the read
    uint8_t tmp[stim300::DATAGRAM_SIZE];
    ssize_t readResult = read(file_handle_, tmp, stim300::DATAGRAM_SIZE);
    savedErrno         = errno;
    if (readResult < 1) {   // error
        if (firstReadError or
            elapsedTime(timeOfLastReadErrMsgSent, currTime) > 5) {
            CFE_EVS_SendEvent(
                IMU_DRIVER_SYSCALL_READ_ERROR, CFE_EVS_EventType_CRITICAL,
                MOON_ERROR_INFO("IMU Driver Serial Port read() error: "
                                "%s. Attempting reconnect."),
                errnoToString(savedErrno));
            reconnect();
            timeOfLastReadErrMsgSent = currTime;
        }
        firstReadError = false;
        return readResult;
    }

    // copy from temporary read buffer into circular buffer
    for (int i = 0; i < stim300::DATAGRAM_SIZE and i < readResult; i++) {
        buffer_.add(tmp[i]);
    }

    return readResult;
}
