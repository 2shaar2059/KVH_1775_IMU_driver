/****************************************************************
 *
 * @file 		imu_driver.h
 *
 * @brief 		The IMU driver header file
 *
 * @version 		1.0
 * @date 		12/15/2021
 *
 * @authors 		Ben Kolligs, Tushaar Jain ...
 * @author 		Carnegie Mellon University, Planetary Robotics Lab
 *
 ****************************************************************/
#ifndef _IMU_DRIVER_H_
#define _IMU_DRIVER_H_

#include <cstring>

#include "imu_data_msg.h"
#include "imu_driver_events.h"
#include "imu_driver_msgs.h"
#include "stim300/driver_stim300.h"
#include "stim300/serial_unix.h"
#include "tbl_manager_table.h"

// for logging
#include <fstream>

#define IMU_DRIVER_PIPE_DEPTH                  32
#define IMU_DRIVER_NUMBER_OF_TABLES            1
#define IMU_DRIVER_TABLE_FILE                  "/cf/imu_config.tbl"
#define IMU_DRIVER_TABLE_OUT_OF_RANGE_ERR_CODE (-1)

/**
 * Global data structure
 */
typedef struct {
    /**
     * Command interface counters
     */
    uint64 CmdCounter;
    uint64 ErrCounter;

    // counters for each stage of processing
    uint64 newPackets;
    uint64 imuCRCErrors;
    uint64 imuInternalErrors;
    uint64 validPackets;
    uint64 sentPacketCount;
    uint64 negativedtCount;
    uint64 excessDtCount;
    bool firstIMUPacket;

    /**
     * RunStatus of the app
     */
    uint32 RunStatus;

    /**
     * Hk/tlm buffer packet
     */
    IMU_DRIVER_HkBuffer_t HkBuf;

    /**
     * IMU_DRIVER data buffer packets
     */
    MOONRANGER_IMUDataBuffer_t IMUBuf;

    /**
     * Operational data
     */
    CFE_SB_PipeId_t CommandPipe;
    CFE_SB_MsgPtr_t MsgPtr;

    /**
     * Initialization Data
     */
    char PipeName[16];
    uint16 PipeDepth;
    CFE_EVS_BinFilter_t EventFilters[IMU_DRIVER_EVENT_COUNTS];
    CFE_TBL_Handle_t TableHandles[IMU_DRIVER_NUMBER_OF_TABLES];
    CFE_ES_CDSHandle_t CDSHandle;

} IMU_DRIVER_AppData_t;

class IMUDriver {
   private:
    SerialUnix serialDriver_{config_.imuPath};

    // timestamp of the previous IMU message sent
    OS_time_t prevIMUmsgTimestamp_{};

    bool unitTesting_;

    std::ofstream imu_data_log;   // for logging
    bool first_time_logging_;
    void logIMUData();
    void initLog();

   public:
    DriverStim300 driverSTIM300_{serialDriver_};
    IMU_DRIVER_AppData_t appData_{};
    TBL_MANAGER_IMU_Driver_Params_t config_ = {
        // NOTE 1: the imuPath length must be less than the length of the
        // designated space for imuPath, now 13
        // NOTE 2: the nominal body-IMU quaternion may appear to be  identity
        // but it's actually a 180 degree flip about the  y axis since the IMU
        // is mounted upside down in moonranger
        "/dev/ttyS0", 125, true, 10, 0.1, false, {0, 0, 1, 0}};

    void run();
    int32 initialize();

    int32 loadValuesFromCommonTable();
    void loadValuesFromTable(
        const TBL_MANAGER_IMU_Driver_Params_t &IMUDriverParams);

    OS_time_t getTimestampFromMsg();
    void setMsgTimestamp(OS_time_t timestamp);
    void sendIMUPacket();
    void processIMUData();

    void processIncomingMessage(CFE_SB_MsgPtr_t msg);
    void processGroundCommand(CFE_SB_MsgPtr_t msg);
    void reportHK();
    void noop();
    void resetCounters();

    explicit IMUDriver(bool unitTesting = false)
        : unitTesting_{unitTesting}, first_time_logging_{true} {
        serialDriver_.resetPath(config_.imuPath);
    }
};

/* Entry point */
extern "C" void IMU_DRIVER_AppMain(void);

#endif   //_IMU_DRIVER_H_ header
