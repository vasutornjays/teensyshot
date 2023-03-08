/*
 *  Definitions for host.c
 */
 
#ifndef __HOST_H
#define __HOST_H

#include <stdint.h>

// Defines
#define HOST_ERROR_FD         -1        // Non existant file descriptor
#define HOST_ERROR_DEV        -2        // Non existant serial device
#define HOST_ERROR_MAX_DEV    -3        // Maximum devices limit 
#define HOST_ERROR_WRITE_SER  -4        // Write error on serial
#define HOST_ERROR_BAD_PK_SZ  -5        // Bad incoming packet size error
#define HOST_ERROR_MAGIC      -6        // Bad magic number received

#define NB_ESC                 8                 // Number of ESCs
#define NB_MAX_ESC             8                 // Max number of ESCs


// COMM Protocol
#define COMM_MAGIC         0x43305735        // Magic number: "teensy35"
// #define ESCPID_COMM_WD_LEVEL      20                // Maximum number of periods without reference refresh
#define COMM_ERROR_MAGIC        -1                // Magic number error code in leet speech
typedef struct {
  uint32_t      magic;                        // Magic number
  int8_t        err[NB_MAX_ESC];          // Last error number
  uint8_t       deg[NB_MAX_ESC];          // ESC temperature (°C)
  uint16_t      cmd[NB_MAX_ESC];          // Current ESC command value
  uint16_t      volt[NB_MAX_ESC];         // Voltage of the ESC power supply (0.01V)
  uint16_t      amp[NB_MAX_ESC];          // ESC current (0.01A)
  int16_t       rpm[NB_MAX_ESC];          // Motor rpm (10 rpm)
} ESCPIDcomm_struct_t;

// Host->teensy communication data structure
// sizeof(Host_comm)=64 to match USB 1.0 buffer size
typedef struct {
  uint32_t      magic;                        // Magic number
  int16_t       dshot[NB_MAX_ESC];        // Velocity reference (10 rpm)
} Hostcomm_struct_t;

// Prototypes

#define HOST_MAX_DEVICES    5                       // Max number of teensys

#define HOST_DEV_SERIALNB     12482610              	// Serial number of the teensy
//#define HOST_DEV_SERIALNB      54887501
#define HOST_DEV_SERIALLG     10                    // Max length of a serial number

#if defined(__linux__)
#define HOST_SERIAL_DEV_DIR   "/dev/serial/by-id/"
#elif defined(__APPLE__)
#define HOST_SERIAL_DEV_DIR   "/dev/"
#else
#define HOST_SERIAL_DEV_DIR   "/dev/"
#endif

#define HOST_BAUDRATE       B115200                 // Serial baudrate
#define HOST_READ_TIMEOUT   5                       // Tenth of second
#define HOST_NB_PING        100000                    // Nb roundtrip communication
#define HOST_STEP_REF       200                     // Velocity reference step size (10 rpm motor)
#define HOST_PERIOD         10000                   // Period of serial exchange (us)
#define HOST_STEP_PERIOD    1                     // Duration of a step (intertions)

char *Host_name_from_serial(  uint32_t );
int   Host_get_fd(            uint32_t );
int   Host_init_port(         uint32_t );
void  Host_release_port(      uint32_t );
int   Host_comm_update(       uint32_t, 
                              int16_t*,
                              ESCPIDcomm_struct_t** );

#endif