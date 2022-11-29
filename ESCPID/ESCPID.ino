/*
 *  Teensyshot:   ESC Controller for teensy
 *
 *  Authors:  Vasutorn Siriyakorn
 *  Date:     Nov 2022
 */



#include <Arduino.h>
#include "DSHOT.h"
#include "ESCCMD.h"
#include "ESCPID.h"

// Globals
// float     g_dshot_val[ESCPID_NB_ESC] = {};
uint16_t  g_host_comm_wd = 0;

int16_t   g_dshot_val[ESCPID_NB_ESC] = {};

bool loop_state_led = 1;

ESCPIDcomm_struct_t g_esc_pid_comm = {
                                  ESCPID_COMM_MAGIC,
                                  {},
                                  {},
                                  {},
                                  {},
                                  {},
                                  {}
                                  };
Hostcomm_struct_t   g_host_comm = {
                                  ESCPID_COMM_MAGIC,
                                  {}
                                  };

int host_comm_update(void){
    static int          i;
    static uint8_t      *ptin   = (uint8_t*)(&g_host_comm),
                        *ptout  = (uint8_t*)(&g_esc_pid_comm);
    static int          ret;
    static int          in_cnt = 0;

    ret = 0;
  
    // Read all incoming bytes available until incoming structure is complete
    while(  ( Serial.available( ) > 0 ) && 
          ( in_cnt < (int)sizeof( g_host_comm ) ) )
    ptin[in_cnt++] = Serial.read( );

    if (in_cnt == (int)sizeof(g_host_comm)){
        // Clear incoming byte
        in_cnt = 0;

        //Check the validity of magic number
        if (g_host_comm.magic != ESCPID_COMM_MAGIC){ // If not match
            // Flush input buffer
            while (Serial.available())
            {
                Serial.read();
            }

            ret = ESCPID_ERROR_MAGIC;
            
        } else {
            // Valid packet received

            // Reset the communication watchdog
            g_host_comm_wd = 0;

            // Update Dshot value
            for( i = 0; i < ESCPID_NB_ESC; i++)
                g_dshot_val[i] = g_host_comm.dshot[i];

            // Update output data structure values
            // If telemetry is invalid, data structure remains unmodified
            for ( i = 0; i < ESCPID_NB_ESC; i++ ) {
                ESCCMD_read_err( i, &g_esc_pid_comm.err[i] );
                ESCCMD_read_cmd( i, &g_esc_pid_comm.cmd[i] );
                ESCCMD_read_deg( i, &g_esc_pid_comm.deg[i] );
                ESCCMD_read_volt( i, &g_esc_pid_comm.volt[i] );
                ESCCMD_read_amp( i, &g_esc_pid_comm.amp[i] );
                ESCCMD_read_rpm( i, &g_esc_pid_comm.rpm[i] );
            }

            // Send data structure to host
            Serial.write( ptout, sizeof( g_esc_pid_comm ) );

            // Force immediate transmission
            Serial.send_now( );
        }
    }

    return ret;

}

void setup() {

    Serial.begin(ESCPID_USB_UART_SPEED); // Init Teensy -> Host serial port

    ESCCMD_init(ESCPID_NB_ESC); // Init ESC dshot and tlm port

    ESCCMD_arm_all( ); // Arming ESCs

    ESCCMD_3D_on( ); // Switch 3D mode on

    ESCCMD_arm_all( ); // Arming ESCs
    
    ESCCMD_start_timer( ); // Start periodic loop

    // Stop all motors
    for (int i = 0; i < ESCPID_NB_ESC; i++ ) {
        ESCCMD_stop( i );
    }

    // Reference watchdog is initially triggered
    g_host_comm_wd = ESCPID_COMM_WD_LEVEL;

    
    pinMode(13, OUTPUT);
    pinMode(14, OUTPUT);
    pinMode(17, OUTPUT);
}

void loop(){

    static int i, ret;
    digitalWrite(13, HIGH);
    // Check for next timer event and tic
    ret = ESCCMD_tic();
    digitalWrite(13, LOW);
    // Bidirectional serial exchange with host
    host_comm_update();

    if ( ret == ESCCMD_TIC_OCCURED )  {

        // Process timer event

        if ( !ESCCMD_read_tlm_status(i) ){
        }

        for(i = 0; i < ESCPID_NB_ESC; i++){
            if ( g_host_comm_wd < ESCPID_COMM_WD_LEVEL ) {
                ret = ESCCMD_throttle(i, g_dshot_val[i]);
            }
        }

        // Update watchdog
        if ( g_host_comm_wd < ESCPID_COMM_WD_LEVEL )  {
        g_host_comm_wd++;
        }
    }
}
