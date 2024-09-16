#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <GCS_MAVLink/GCS_config.h>

#ifndef AP_GPS_ENABLED
#define AP_GPS_ENABLED 1
#endif

#if AP_GPS_ENABLED
/**
   maximum number of GPS instances available on this platform. If more
   than 1 then redundant sensors may be available
 */
#ifndef GPS_MAX_RECEIVERS
#define GPS_MAX_RECEIVERS 2 // maximum number of physical GPS sensors allowed - does not include virtual GPS created by blending receiver data
#endif
#if !defined(GPS_MAX_INSTANCES)
#if GPS_MAX_RECEIVERS > 1
#define GPS_MAX_INSTANCES  (GPS_MAX_RECEIVERS + 1) // maximum number of GPS instances including the 'virtual' GPS created by blending receiver data
#else
#define GPS_MAX_INSTANCES 1
#endif // GPS_MAX_RECEIVERS > 1
#endif // GPS_MAX_INSTANCES

#if GPS_MAX_RECEIVERS <= 1 && GPS_MAX_INSTANCES > 1
#error "GPS_MAX_INSTANCES should be 1 for GPS_MAX_RECEIVERS <= 1"
#endif

#if GPS_MAX_INSTANCES > GPS_MAX_RECEIVERS
#define GPS_BLENDED_INSTANCE GPS_MAX_RECEIVERS  // the virtual blended GPS is always the highest instance (2)
#endif
#endif

#ifndef AP_GPS_BACKEND_DEFAULT_ENABLED
#define AP_GPS_BACKEND_DEFAULT_ENABLED AP_GPS_ENABLED
#endif

#ifndef AP_GPS_BLENDED_ENABLED
#define AP_GPS_BLENDED_ENABLED AP_GPS_BACKEND_DEFAULT_ENABLED && defined(GPS_BLENDED_INSTANCE)
#endif

#ifndef AP_GPS_DRONECAN_ENABLED
#define AP_GPS_DRONECAN_ENABLED 0 //AP_GPS_BACKEND_DEFAULT_ENABLED && HAL_ENABLE_DRONECAN_DRIVERS
#endif

#ifndef AP_GPS_ERB_ENABLED
  #define AP_GPS_ERB_ENABLED 0 //AP_GPS_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_GPS_GSOF_ENABLED
  #define AP_GPS_GSOF_ENABLED AP_GPS_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_GPS_MAV_ENABLED
  #define AP_GPS_MAV_ENABLED 0 //AP_GPS_BACKEND_DEFAULT_ENABLED && HAL_GCS_ENABLED
#endif

#ifndef HAL_MSP_GPS_ENABLED
#define HAL_MSP_GPS_ENABLED 0 //AP_GPS_BACKEND_DEFAULT_ENABLED && HAL_MSP_SENSORS_ENABLED
#endif

#ifndef AP_GPS_NMEA_ENABLED
  #define AP_GPS_NMEA_ENABLED AP_GPS_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_GPS_NMEA_UNICORE_ENABLED
  #define AP_GPS_NMEA_UNICORE_ENABLED AP_GPS_NMEA_ENABLED
#endif

#ifndef AP_GPS_NOVA_ENABLED
  #define AP_GPS_NOVA_ENABLED 0 //AP_GPS_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_GPS_SBF_ENABLED
  #define AP_GPS_SBF_ENABLED 0 //AP_GPS_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_GPS_SBP_ENABLED
  #define AP_GPS_SBP_ENABLED 0 //AP_GPS_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_GPS_SBP2_ENABLED
   #define AP_GPS_SBP2_ENABLED 0 //AP_GPS_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_GPS_SIRF_ENABLED
  #define AP_GPS_SIRF_ENABLED 0 //AP_GPS_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_GPS_UBLOX_ENABLED
  #define AP_GPS_UBLOX_ENABLED 0 //AP_GPS_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_GPS_RTCM_DECODE_ENABLED
  #define AP_GPS_RTCM_DECODE_ENABLED BOARD_FLASH_SIZE > 1024
#endif

#ifndef HAL_GPS_COM_PORT_DEFAULT
#define HAL_GPS_COM_PORT_DEFAULT 1
#endif
