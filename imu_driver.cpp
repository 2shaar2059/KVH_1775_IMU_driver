/****************************************************************
 *
 * @file 		imu_driver.cpp
 *
 * @brief 		The IMU driver runtime application file
 *
 * @version 		1.0
 * @date 		12/15/2021
 *
 * @authors 		Ben Kolligs, Tushaar Jain ...
 * @author 		Carnegie Mellon University, Planetary Robotics Lab
 *
 ****************************************************************/

extern "C" {
#include "cfe.h"
}

#include <chrono>
#include <cmath>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iomanip>

#include "EigenWrapper.h"
#include "common_utils.h"
#include "hs_msgdefs.h"
#include "imu_driver.h"
#include "imu_driver_perfids.h"
#include "imu_driver_version.h"
#include "moonranger_generic_msgs.h"
#include "moonranger_msgids.h"
#include "moonranger_output_strings.h"
#include "tbl_manager.h"

using namespace std::chrono_literals;

// Global data
static IMUDriver imuDriver;

// Entry point function
void IMU_DRIVER_AppMain(void) {
    imuDriver.run();
}

// Defining the member functions for IMUDriver
void IMUDriver::run() {
    // Register the app with executive services
    CFE_ES_RegisterApp();

    // Create the first Performance log entry
    CFE_ES_PerfLogEntry(IMU_DRIVER_PERF_ID);

    // Initialize the application
    int32 status = initialize();
    if (status != CFE_SUCCESS) {
        appData_.RunStatus = CFE_ES_RunStatus_APP_ERROR;
    }

    while (CFE_ES_RunLoop(&appData_.RunStatus)) {
        if (driverSTIM300_.tryReadingNewPacket()) {
            imuDriver.processIMUData();
        }

        CFE_ES_PerfLogExit(IMU_DRIVER_PERF_ID);   // Performance log exit stamp

        /* Keep polling for receipt of a command packet. If there is none
         * avaible, CFE_SB_RcvMsg() will not block and returns CFE_SB_NO_MESSAGE
         */
        status =
            CFE_SB_RcvMsg(&appData_.MsgPtr, appData_.CommandPipe, CFE_SB_POLL);

        CFE_ES_PerfLogEntry(IMU_DRIVER_PERF_ID);

        if (status == CFE_SUCCESS) {
            processIncomingMessage(appData_.MsgPtr);
        } else if (status == CFE_SB_NO_MESSAGE) {
            // no new nessage to be received
        } else {
            CFE_EVS_SendEvent(IMU_DRIVER_PIPE_ERR_EID, CFE_EVS_EventType_ERROR,
                              "IMU_DRIVER: SB Pipe Read Error, App Will Exit");
            appData_.RunStatus = CFE_ES_RunStatus_APP_ERROR;
        }
        /* Thread cancellation point/give up CPU */
        OS_TaskDelay(0);
    }
    CFE_ES_WriteToSysLog("%s\n", RED_STRING("EXITING IMU_DRIVER APP"));

    // Exit the app
    CFE_ES_PerfLogExit(IMU_DRIVER_PERF_ID);
    CFE_ES_ExitApp(appData_.RunStatus);
}

int32 IMUDriver::initialize() {
    // Start the app
    appData_.RunStatus         = CFE_ES_RunStatus_APP_RUN;
    appData_.CmdCounter        = 0;
    appData_.ErrCounter        = 0;
    appData_.newPackets        = 0;
    appData_.imuCRCErrors      = 0;
    appData_.imuInternalErrors = 0;
    appData_.validPackets      = 0;
    appData_.firstIMUPacket    = true;
    appData_.sentPacketCount   = 0;
    appData_.negativedtCount   = 0;
    appData_.excessDtCount     = 0;
    appData_.PipeDepth         = IMU_DRIVER_PIPE_DEPTH;
    strcpy(appData_.PipeName, "IMU_CMD_PIPE");

    // Initialize event filter table
    int event_ids[IMU_DRIVER_EVENT_COUNTS - 1] = {
        IMU_DRIVER_STARTUP_INF_EID,
        IMU_DRIVER_COMMAND_ERR_EID,
        IMU_DRIVER_COMMANDNOP_INF_EID,
        IMU_DRIVER_COMMANDRST_INF_EID,
        IMU_DRIVER_INVALID_MSGID_ERR_EID,
        IMU_DRIVER_LEN_ERR_EID,
        IMU_DRIVER_PIPE_ERR_EID,
        IMU_DRIVER_START_INF_EID,
        IMU_DRIVER_STOP_INF_EID,
        IMU_DRIVER_TBL_ACCESS_EID,
        IMU_DRIVER_NEGATIVE_TIMESTEP,
        IMU_DRIVER_EXCESSIVE_TIMESTEP,
        IMU_DRIVER_STIM_HARDWARE_ERR_EID,
        IMU_DRIVER_STIM_CRC_ERR_EID,
        IMU_DRIVER_SYSCALL_ISTTY_ERROR,
        IMU_DRIVER_SYSCALL_TCGETATTR_ERROR,
        IMU_DRIVER_SYSCALL_CFSETIOSPEED_ERROR,
        IMU_DRIVER_SYSCALL_TCSETATTR_ERROR,
        IMU_DRIVER_SYSCALL_OPEN_ERROR,
        IMU_DRIVER_SYSCALL_OPEN_SUCCESS,
        IMU_DRIVER_SYSCALL_POLL_ERROR,
        IMU_DRIVER_SYSCALL_READ_ERROR,
        IMU_DRIVER_SYSCALL_CLOSE_ERROR};
    int arrayIndex = 0;
    for (int event_id : event_ids) {
        appData_.EventFilters[arrayIndex].EventID = event_id;
        appData_.EventFilters[arrayIndex].Mask    = 0x0000;
        arrayIndex++;
    }

    // Register the events
    int32 status =
        CFE_EVS_Register(appData_.EventFilters, IMU_DRIVER_EVENT_COUNTS,
                         CFE_EVS_EventFilter_BINARY);
    if (status != CFE_SUCCESS) {
        CFE_ES_WriteToSysLog(
            "IMU Driver: Error Registering Events, RC = 0x%08lX\n",
            (unsigned long)status);
        return status;
    }

    // Create Software Bus message pipe
    status = CFE_SB_CreatePipe(&appData_.CommandPipe, appData_.PipeDepth,
                               appData_.PipeName);
    if (status != CFE_SUCCESS) {
        CFE_ES_WriteToSysLog("IMU Driver: Error creating pipe, RC = 0x%08lX\n",
                             (unsigned long)status);
        return status;
    }

    status = CFE_SB_Subscribe(IMU_DRIVER_CMD_MID, appData_.CommandPipe);
    if (status != CFE_SUCCESS) {
        CFE_ES_WriteToSysLog(
            "IMU Driver: Error Subscribing to Command, RC = 0x%08lX\n",
            (unsigned long)status);

        return status;
    }

    status = CFE_SB_Subscribe(IMU_DRIVER_SEND_HK_MID, appData_.CommandPipe);

    if (status != CFE_SUCCESS) {
        CFE_ES_WriteToSysLog(
            "IMU Driver: Error Subscribing to HK Request, RC = 0x%08lX\n",
            (unsigned long)status);

        return status;
    }

    // Subscribe to table update message
    status =
        CFE_SB_Subscribe(TBL_MANAGER_SEND_UPDATE_MID, appData_.CommandPipe);
    if (status != CFE_SUCCESS) {
        CFE_ES_WriteToSysLog(
            "IMU Driver: Error Subscribing to table update message, RC = "
            "0x%08lX\n",
            (unsigned long)status);
        return status;
    }

    // try loading from table
    status = loadValuesFromCommonTable();
    if (status != CFE_SUCCESS) {
        CFE_ES_WriteToSysLog(
            MOON_ERROR_INFO("IMU Driver: unable to load information from "
                            "table manager, RC = 0x%08lX"),
            (unsigned long)status);
        return status;
    } else {
        CFE_ES_WriteToSysLog(
            MOON_SUCCESS_INFO("Initialized IMU Driver Params: imuPath: %s, "
                              "tryToRecconect: %s "
                              "maxReconnectionAttempts: %d timestepThresh: %f"),
            config_.imuPath, config_.tryToRecconect ? "true" : "false",
            config_.maxReconnectionAttempts, config_.timestepThresh);
    }

    // once write_to_file param is loaded from table, init the log
    initLog();

    // Initialize housekeeping packet (clear user data area).
    CFE_SB_InitMsg(&appData_.HkBuf.MsgHdr, IMU_DRIVER_HK_TLM_MID,
                   sizeof(appData_.HkBuf), true);

    // Initialize imu packet
    CFE_SB_InitMsg(&appData_.IMUBuf.MsgHdr, MOONRANGER_IMU_DATA_MID,
                   sizeof(appData_.IMUBuf), true);

    if (not unitTesting_) {
        /*keep trying to reconnect if we should (config_.tryToRecconect is true)
         * and IMU isnt already connected*/
        for (int attempts = 0;
             config_.tryToRecconect and not serialDriver_.isPortOpened() and
             attempts < config_.maxReconnectionAttempts;
             attempts++) {
            // Sleep for some 5000ms before attempting a reconnect
            OS_TaskDelay(5000);
            serialDriver_.reconnect();
        }
        if (not serialDriver_.isPortOpened()) {   // still not connected
            CFE_EVS_SendEvent(
                IMU_DRIVER_SYSCALL_OPEN_ERROR, CFE_EVS_EventType_ERROR,
                MOON_ERROR_INFO(
                    "STIM300 IMU not connected, but app still running"));
        }
    }

    // register with HS app to make sure this app gets automatically restarted
    // when it crashes
    CFE_SB_CmdHdr_t EnableHs;
    CFE_SB_InitMsg(&EnableHs, HS_CMD_MID, sizeof(CFE_SB_CmdHdr_t), true);
    CFE_SB_SetCmdCode((CFE_SB_Msg_t *)&EnableHs, HS_ENABLE_APPMON_CC);
    CFE_SB_GenerateChecksum((CFE_SB_Msg_t *)&EnableHs);
    CFE_SB_SendMsg((CFE_SB_Msg_t *)&EnableHs);

    // Send the initialization message
    CFE_EVS_SendEvent(IMU_DRIVER_STARTUP_INF_EID, CFE_EVS_EventType_INFORMATION,
                      MOON_SUCCESS_INFO("IMU Driver App Initialized. %s"),
                      IMU_DRIVER_VERSION_STRING);

    return CFE_SUCCESS;
}

void IMUDriver::loadValuesFromTable(
    const TBL_MANAGER_IMU_Driver_Params_t &IMUDriverParams) {
    config_ = IMUDriverParams;
    driverSTIM300_.setDevicePath(config_.imuPath);
    CFE_EVS_SendEvent(
        IMU_DRIVER_TBL_ACCESS_EID, CFE_EVS_EventType_INFORMATION,
        MOON_SUCCESS_INFO("Initialized IMU Driver Params: imuPath: %s, "
                          "tryToRecconect: %s "
                          "maxReconnectionAttempts: %d timestepThresh: %f, "
                          "write_to_file: %d"),
        config_.imuPath, config_.tryToRecconect ? "true" : "false",
        config_.maxReconnectionAttempts, config_.timestepThresh,
        static_cast<int>(config_.write_to_file));
}

int32 IMUDriver::loadValuesFromCommonTable() {
    // Read in table from table manager
    TBL_MANAGER_Param_Tbl_t ParamTable;
    int32 status = TBL_MANAGER_AccessTable(&ParamTable);
    if (status != CFE_SUCCESS) {
        CFE_EVS_SendEvent(
            IMU_DRIVER_TBL_ACCESS_EID, CFE_EVS_EventType_ERROR,
            MOON_ERROR_INFO("IMU Driver: unable to load information from "
                            "table manager, RC = 0x%08lX"),
            (unsigned long)status);
    } else {
        loadValuesFromTable(ParamTable.IMUDriverParams);
    }
    return status;
}

void IMUDriver::sendIMUPacket() {
    Vector3 specific_force_IMU   = driverSTIM300_.getAccData();
    Vector3 angular_velocity_IMU = driverSTIM300_.getGyroData();

    Quaternion BODY_q_IMU =
        Quaternion{config_.BODY_q_IMU[0], config_.BODY_q_IMU[1],
                   config_.BODY_q_IMU[2], config_.BODY_q_IMU[3]};

    Vector3 angular_velocity_b = BODY_q_IMU * angular_velocity_IMU;
    Vector3 specific_force_b   = BODY_q_IMU * specific_force_IMU;

    // Populate the message before sending
    auto *data               = &(appData_.IMUBuf.IMUTlm.data);
    data->angular_velocity_x = angular_velocity_b.x();
    data->angular_velocity_y = angular_velocity_b.y();
    data->angular_velocity_z = angular_velocity_b.z();
    data->specific_force_x   = specific_force_b.x();
    data->specific_force_y   = specific_force_b.y();
    data->specific_force_z   = specific_force_b.z();

    OS_time_t currIMUmsgTimestamp;
    CFE_PSP_GetTime(&currIMUmsgTimestamp);
    appData_.IMUBuf.IMUTlm.data.timeStamp = currIMUmsgTimestamp;

    // Send the first message so that we can start calculating elapsed time
    // between messages
    if (appData_.firstIMUPacket) {
        appData_.sentPacketCount++;
        CFE_SB_TimeStampMsg(&appData_.IMUBuf.MsgHdr);
        CFE_SB_SendMsg(&(appData_.IMUBuf.MsgHdr));
        appData_.firstIMUPacket = false;
    } else {
        double timestep =
            elapsedTime(prevIMUmsgTimestamp_, currIMUmsgTimestamp);
        if (timestep < 0) {
            appData_.negativedtCount++;
            // critical error because this could mean timers arent
            // monotonically increasing
            CFE_EVS_SendEvent(IMU_DRIVER_NEGATIVE_TIMESTEP,
                              CFE_EVS_EventType_CRITICAL,
                              "IMU Driver received an IMU data packet with "
                              "negative timestep! %lf",
                              timestep);
            // dont return despite error; still need to update previous
            // timestamp

        } else if (timestep > config_.timestepThresh) {
            appData_.excessDtCount++;
            // non-critical error because this happens occasionally
            CFE_EVS_SendEvent(
                IMU_DRIVER_EXCESSIVE_TIMESTEP, CFE_EVS_EventType_ERROR,
                "IMU Driver received a packet with an "
                "excessive timestep %f -> %f (dt = %lf s) !",
                elapsedTime({0, 0}, prevIMUmsgTimestamp_),
                elapsedTime({0, 0}, currIMUmsgTimestamp), timestep);
            // not sending a message since the timestap might be HUGE
            // dont return despite error; still need to update previous values

        } else {
            appData_.sentPacketCount++;
            CFE_SB_TimeStampMsg(&appData_.IMUBuf.MsgHdr);
            CFE_SB_SendMsg(&(appData_.IMUBuf.MsgHdr));
        }
    }
    prevIMUmsgTimestamp_ = currIMUmsgTimestamp;
}

void IMUDriver::processIMUData() {
    appData_.newPackets++;
    bool error = false;
    if (!driverSTIM300_.isChecksumGood()) {
        appData_.imuCRCErrors++;
        CFE_EVS_SendEvent(IMU_DRIVER_STIM_CRC_ERR_EID, CFE_EVS_EventType_ERROR,
                          "STIM300 CRC error");
        error = true;
    } else if (!driverSTIM300_.isSensorStatusGood()) {
        appData_.imuInternalErrors++;
        CFE_EVS_SendEvent(IMU_DRIVER_STIM_HARDWARE_ERR_EID,
                          CFE_EVS_EventType_ERROR,
                          "STIM300: Internal hardware error");
        error = true;
    }
    if (not error) {
        appData_.validPackets++;
        sendIMUPacket();
        logIMUData();
    }
}

void IMUDriver::processIncomingMessage(CFE_SB_MsgPtr_t msg) {
    CFE_SB_MsgId_t MsgId = CFE_SB_GetMsgId(msg);
    switch (MsgId) {
        case IMU_DRIVER_CMD_MID: {
            processGroundCommand(msg);
            break;
        }
        case IMU_DRIVER_SEND_HK_MID: {
            reportHK();
            break;
        }
        case TBL_MANAGER_SEND_UPDATE_MID: {
            loadValuesFromCommonTable();
            appData_.CmdCounter++;
            break;
        }
        default: {
            CFE_EVS_SendEvent(IMU_DRIVER_INVALID_MSGID_ERR_EID,
                              CFE_EVS_EventType_ERROR,
                              "IMU_DRIVER: invalid command packet, MID = 0x%x",
                              (unsigned int)CFE_SB_MsgIdToValue(MsgId));
            break;
        }
    }
}

void IMUDriver::processGroundCommand(CFE_SB_MsgPtr_t msg) {
    uint16 CommandCode = CFE_SB_GetCmdCode(msg);
    switch (CommandCode) {
        case IMU_DRIVER_NOOP_CC: {
            if (VerifyMsgLength(msg, sizeof(MOONRANGER_Noop_t),
                                IMU_DRIVER_LEN_ERR_EID,
                                &(appData_.ErrCounter))) {
                noop();
            }
            break;
        }
        case IMU_DRIVER_RESET_COUNTERS_CC: {
            if (VerifyMsgLength(msg, sizeof(MOONRANGER_ResetCounters_t),
                                IMU_DRIVER_LEN_ERR_EID,
                                &(appData_.ErrCounter))) {
                resetCounters();
            }
            break;
        }
        default: {
            CFE_EVS_SendEvent(
                IMU_DRIVER_COMMAND_ERR_EID, CFE_EVS_EventType_ERROR,
                "IMU_DRIVER: Invalid ground command code: %d", CommandCode);
            break;
        }
    }
}

void IMUDriver::reportHK() {
    // populate the telemetry packet with the command counters
    appData_.HkBuf.HkTlm.Payload.CommandCounter      = appData_.CmdCounter;
    appData_.HkBuf.HkTlm.Payload.CommandErrorCounter = appData_.ErrCounter;
    appData_.HkBuf.HkTlm.Payload.newPackets          = appData_.newPackets;
    appData_.HkBuf.HkTlm.Payload.imuCRCErrors        = appData_.imuCRCErrors;
    appData_.HkBuf.HkTlm.Payload.imuInternalErrors = appData_.imuInternalErrors;
    appData_.HkBuf.HkTlm.Payload.validPackets      = appData_.validPackets;
    appData_.HkBuf.HkTlm.Payload.sentPacketCount   = appData_.sentPacketCount;
    appData_.HkBuf.HkTlm.Payload.negativedtCount   = appData_.negativedtCount;
    appData_.HkBuf.HkTlm.Payload.excessDtCount     = appData_.excessDtCount;

    // send the housekeeping telemetry packet
    CFE_SB_TimeStampMsg(&appData_.HkBuf.MsgHdr);
    CFE_SB_SendMsg(&appData_.HkBuf.MsgHdr);
}

void IMUDriver::noop() {
    // send a no op command event
    CFE_EVS_SendEvent(IMU_DRIVER_COMMANDNOP_INF_EID,
                      CFE_EVS_EventType_INFORMATION,
                      "IMU_DRIVER: NOOP Command");
    appData_.CmdCounter++;
}

void IMUDriver::resetCounters() {
    // reset counters
    appData_.CmdCounter        = 0;
    appData_.ErrCounter        = 0;
    appData_.newPackets        = 0;
    appData_.imuCRCErrors      = 0;
    appData_.imuInternalErrors = 0;
    appData_.validPackets      = 0;
    appData_.sentPacketCount   = 0;
    appData_.negativedtCount   = 0;
    appData_.excessDtCount     = 0;

    // send reset event
    CFE_EVS_SendEvent(IMU_DRIVER_COMMANDRST_INF_EID,
                      CFE_EVS_EventType_INFORMATION,
                      "IMU_DRIVER: RESET Command");
    // Dont increment command counter since the whole point of resetcounters
    // command was to zero out the counters
}

/* Below Functions are only for logging
 * Courtesy of Tushaar Jain / Samuel Ong */

void IMUDriver::initLog() {
    if (config_.write_to_file) {
        auto in_time_t = std::chrono::system_clock::to_time_t(
            std::chrono::system_clock::now());
        std::stringstream datetime;
        datetime << std::put_time(std::localtime(&in_time_t),
                                  "%m-%d-%Y-%H-%M-%S");

        // opening log in home directory instead of sd/ because /sd is slow
        std::string log_file_path =
            "/home/morphin/imu_log_" + std::string(datetime.str()) + ".csv";
        imu_data_log.open(log_file_path);

        if (imu_data_log.good()) {
            CFE_ES_WriteToSysLog(
                MOON_SUCCESS_INFO("IMU Driver log file at: %s"),
                log_file_path.c_str());
        } else {
            CFE_ES_WriteToSysLog(
                MOON_WARNING_INFO("IMU Driver: Unable to open log files"));
        }
    }
}

void IMUDriver::logIMUData() {
    if (config_.write_to_file) {
        if (imu_data_log.good()) {
            if (first_time_logging_) { // write header
                imu_data_log
                << "time [s]" << "," 
                << "Sent Packet counter" << ","
                << "Gyro x [rad/s]" << "," << "Gyro y [rad/s]" << "," << "Gyro z [rad/s]" << ","
                << "Accel x [m/s^2]" << "," << "Accel y [m/s^2]" << "," << "Accel z [m/s^2]" << ","
                << std::endl;   // use std::endl for newline and flush

                first_time_logging_=false;
            }
            imu_data_log << std::setprecision(10);
            imu_data_log << get_CFS_monotonic_time()  << ","
                         // << prevIMUmsgTimestam p_.seconds << ","
                         // << prevIMUmsgTimestamp_.microsecs << ","
                         << appData_.sentPacketCount << ","
                         << appData_.IMUBuf.IMUTlm.data.angular_velocity_x << ","
                         << appData_.IMUBuf.IMUTlm.data.angular_velocity_y << ","
                         << appData_.IMUBuf.IMUTlm.data.angular_velocity_z << "," 
                         << appData_.IMUBuf.IMUTlm.data.specific_force_x << "," 
                         << appData_.IMUBuf.IMUTlm.data.specific_force_y << "," 
                         << appData_.IMUBuf.IMUTlm.data.specific_force_z << "," 
                         << std::endl;   // use std::endl for newline and flush
        }
    }
}
