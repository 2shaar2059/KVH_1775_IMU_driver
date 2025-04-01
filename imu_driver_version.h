/****************************************************************
 * 
 * @file 		imu_driver_version.h
 * 
 * @brief 		Header file specifying version numbers
 * 
 * @version 	1.0
 * @date 		11/16/2021
 * 
 * @authors 	Ben Kolligs, ...
 * @author 		Carnegie Mellon University, Planetary Robotics Lab
 * 
 ****************************************************************/
#ifndef IMU_DRIVER_VERSION_H
#define IMU_DRIVER_VERSION_H

/* Development Build Macro Definitions */

#define IMU_DRIVER_BUILD_NUMBER 0 /*!< Development Build: Number of commits since baseline */
#define IMU_DRIVER_BUILD_BASELINE "0.1.0" /*!< Development Build: git tag that is the base for the current development */

/* Version Macro Definitions */

#define IMU_DRIVER_MAJOR_VERSION 0 /*!< @brief ONLY APPLY for OFFICIAL releases. Major version number. */
#define IMU_DRIVER_MINOR_VERSION 1 /*!< @brief ONLY APPLY for OFFICIAL releases. Minor version number. */
#define IMU_DRIVER_REVISION      0 /*!< @brief ONLY APPLY for OFFICIAL releases. Revision version number. */
#define IMU_DRIVER_MISSION_REV   0 /*!< @brief ONLY USED by MISSION Implementations. Mission revision */

#define IMU_DRIVER_STR_HELPER(x) #x /*!< @brief Helper function to concatenate strings from integer macros */
#define IMU_DRIVER_STR(x)        IMU_DRIVER_STR_HELPER(x) /*!< @brief Helper function to concatenate strings from integer macros */

/*! @brief Development Build Version Number. 
 * @details Baseline git tag + Number of commits since baseline. @n
 * See @ref cfsversions for format differences between development and release versions.
 */
#define IMU_DRIVER_VERSION IMU_DRIVER_BUILD_BASELINE

/*! @brief Development Build Version String.
 * @details Reports the current development build's baseline, number, and name. Also includes a note about the latest official version. @n
 * See @ref cfsversions for format differences between development and release versions. 
*/          
#define IMU_DRIVER_VERSION_STRING                                                          \
    " IMU Driver App DEVELOPMENT BUILD "                                                     \
    IMU_DRIVER_VERSION                                                                     \
    ", Last Official Release: v1.1.0"   /* For full support please use this version */

#endif /* IMU_DRIVER_VERSION_H */

/************************/
/*  End of File Comment */
/************************/