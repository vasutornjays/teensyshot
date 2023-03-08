/*
 *  Teensyshot:   ESC Controller for teensy
 *
 *  Authors:  Vasutorn Siriyakorn
 *  Date:     Nov 2022
 */

#include <Arduino.h>
#include "DSHOT.h"
#include "ESCCMD.h"
#include "teensyshot_firmware.h"

// Globals
// float     g_dshot_val[ESCPID_NB_ESC] = {};
uint16_t g_host_comm_wd = 0;

int16_t g_dshot_val[ESCPID_NB_ESC] = {};

ESCPIDcomm_struct_t g_esc_pid_comm = {
    ESCPID_COMM_MAGIC,
    {},
    {},
    {},
    {},
    {},
    {}};
Hostcomm_struct_t g_host_comm = {
    ESCPID_COMM_MAGIC,
    {}};

int host_comm_update(void)
{
    static int i;
    static uint8_t *ptin = (uint8_t *)(&g_host_comm),
                   *ptout = (uint8_t *)(&g_esc_pid_comm);
    static int ret;
    static int in_cnt = 0;

    ret = 0;

    // Read all incoming bytes available until incoming structure is complete
    while ((Serial.available() > 0) && (in_cnt < (int)sizeof(g_host_comm)))
    {
        ptin[in_cnt++] = Serial.read();
    }

    if (in_cnt == (int)sizeof(g_host_comm))
    {
        // Clear incoming byte
        in_cnt = 0;

        // Check the validity of magic number
        if (g_host_comm.magic != ESCPID_COMM_MAGIC)
        { // If not match
            // Flush input buffer
            while (Serial.available())
            {
                Serial.read();
            }

            ret = ESCPID_ERROR_MAGIC;
        }
        else
        {
            // Valid packet received

            // Reset the communication watchdog
            g_host_comm_wd = 0;

            // Update Dshot value
            for (i = 0; i < ESCPID_NB_ESC; i++)
                g_dshot_val[i] = g_host_comm.dshot[i];

            // Update output data structure values
            // If telemetry is invalid, data structure remains unmodified
            for (i = 0; i < ESCPID_NB_ESC; i++)
            {
                ESCCMD_read_err(i, &g_esc_pid_comm.err[i]); // ESCCMD_last_error
                ESCCMD_read_cmd(i, &g_esc_pid_comm.cmd[i]); 
                ESCCMD_read_deg(i, &g_esc_pid_comm.deg[i]);
                ESCCMD_read_volt(i, &g_esc_pid_comm.volt[i]);
                ESCCMD_read_amp(i, &g_esc_pid_comm.amp[i]);
                ESCCMD_read_rpm(i, &g_esc_pid_comm.rpm[i]);
            }

            // Send data structure to host
            Serial.write(ptout, sizeof(g_esc_pid_comm));

            // Force immediate transmission
            Serial.send_now();
        }
    }

    return ret;
}

void setup()
{

    Serial.begin(ESCPID_USB_UART_SPEED); // Init Teensy -> Host serial port

    ESCCMD_init(ESCPID_NB_ESC); // Init ESC dshot and tlm port

    ESCCMD_arm_all(); // Arming ESCs

    ESCCMD_3D_on(); // Switch 3D mode on

    ESCCMD_arm_all(); // Arming ESCs

    ESCCMD_start_timer(); // Start periodic loop
    // * loop for ESCCMD_tic => Rate of Dshot send to ESC
    // * ESCCMD_TIMER_PERIOD = 2500 us

    // Stop all motors
    for (int i = 0; i < ESCPID_NB_ESC; i++)
    {
        ESCCMD_stop(i);
    }

    // Reference watchdog is initially triggered
    g_host_comm_wd = ESCPID_COMM_WD_LEVEL;
}

void loop()
{

    static int i, ret;
    // Check for next timer event and tic
    ret = ESCCMD_tic();
    // * Check timer flag
        // ret = 0 if timer flag is not set
    // * Read telemetry packet
    // * Extract buffer data to struct
        // Check CRC error
        // Chect temperature
    // * Check telemetry packet pending
        // If exceed return packet will lost
        // If number of lost packet exceed return -10
    // * >> Process Tics 
    // * Check if tics pending (tic is increase by timer)
        // ^ Decrease peding tic
        // ^ Increase tic counter
            // if ESCCMD_tic_counter % ESCCMD_TLM_PER == 0 reset
                // ESCCMD_tlm_lost_cnt and ESCCMD_CRC_errors
        // ^ If ESCCMD_init_flag is not set
            // ~ Return ESCCMD_TIC_OCCURED (1) with esc error ESCCMD_ERROR_INIT (-3)
        // ^ If ESCCMD_state is not armed
            // ~ Return ESCCMD_TIC_OCCURED (1) with esc error ESCCMD_ERROR_SEQ (-2)
        // ^ Check Watchdog
            // If excess stop ESC and set ESCCMD_state to STOP
            // else increase WD count
        // ^ Send Dshot command and request telemetry
            // If error return  ESCCMD_TIC_OCCURED (1) with esc error ESCCMD_ERROR_DSHOT (-1)
            // Else delay between two consecutive DSHOT transmissions (us)
                // Increase telemetry pending ESCCMD_tlm_pend
            // ~ Return ESCCMD_TIC_OCCURED (1)
    // ~ Return 0 if not match any case
        
    // Bidirectional serial exchange with host
    host_comm_update();
    // * Read serial data from host utill 
    // * If n byte == sizeof(g_host_comm)
        // check magic number (sync check)
            // If not match flush all incoming serial data
            // Else
                // ^ Reset g_host_comm_wd 
                // ^ Update Dshot value
                // ^ Costruct TLM packet and send to host
                
    if (ret == ESCCMD_TIC_OCCURED) // If tic occured
    {
        // Process timer event

        if (!ESCCMD_read_tlm_status(i)) // telemetry is valid and active (return 0)
        {
        }

        for (i = 0; i < ESCPID_NB_ESC; i++)
        {
            if (g_host_comm_wd < ESCPID_COMM_WD_LEVEL)
            {
                ret = ESCCMD_throttle(i, g_dshot_val[i]);
                // * Check ESC is initialized and timer started
                // * Check ESC is ARMED
                // * Check ESC Mode
                    // Convert g_dshot_val[i] to ESCCMD wrt ESC mode
                // * If ESC state is not started
                    // Start ESC and set telemetry bit request to 1 
                // * Check is ESCCMD_throttle_wd exceed limit (increased in ESCCMD_tic)
                    // if exceed reset all ESC error
                // * Reset ESCCMD_throttle_wd (Dshot command is updated)
            } else {
                ret = ESCCMD_throttle(i, 0); // If cannot get update cmd val from host
            }
        }

        // Update watchdog
        if (g_host_comm_wd < ESCPID_COMM_WD_LEVEL)
        {
            g_host_comm_wd++;
        }
    }
}
