/*
    File Description:
        To avoid circular dependencies, some defines need to be set here so as to allow them to be 
        included in the traceMacros.h header file and by extension the FreeRTOSConfig.h header file
*/

#ifndef _COMMON_DEFINES_H_
#define _COMMON_DEFINES_H_

// **************************** CONFIG DEFINES ***************************//
//#define TRACE_CONFIG                        // Define this to enable trace features, must comment out ESP_TRACE_CONFIG Define 
#define ESP_TRACE_CONFIG                    // Define this to enable trace features in microseconds, must comment out TRACE_CONFIG

#ifdef TRACE_CONFIG
#define TRACE_ARRAY_SIZE                    configTICK_RATE_HZ * 3
#endif

#define USE_TBS                             0  // Set to 1 to use TBS instead of the aperiodic server
#define USE_WCET_CHECKS                     1
#define USE_DEADLINE_CHECKS                 0

#if USE_TBS == 1

#if USE_DEADLINE_CHECKS == 1
#define USE_DEADLINE_CHECKS_TBS             0
#endif 

#if USE_WCET_CHECKS == 1
#define USE_WCET_CHECKS_TBS                 0
#endif

#endif
// *********************************************************************** //

#endif // _COMMON_DEFINES_H_