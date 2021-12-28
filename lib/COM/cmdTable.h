#pragma once
#ifndef cmdTable_h_
#define cmdTable_h_

#include "Arduino.h"

#define TOTAL_CMD                       23               

//Power CMD
#define COM_CMD_SLEEP                   0x87
#define COM_CMD_RESTART                 0x88
#define COM_CMD_PING                    0x89

//Manufacturing CMD
#define COM_CMD_UUID                    0x51
#define COM_CMD_MODEL                   0x52
#define COM_CMD_FIRMWARE                0x53
#define COM_CMD_VERSION                 0x54
#define COM_CMD_RD_SN                   0x55
#define COM_CMD_WR_SN                   0x56
#define COM_CMD_WR_HOST                 0x57

// Sensor & data flag
#define COM_CMD_LEVEL                   0x61        // water height or level in tank
#define COM_CMD_VOLUME                  0x62        // water volume in tank
#define COM_CMD_RAW_DATA                0x63        // raw data of adc
#define COM_CMD_SF                      0x64        // status of sensor
#define COM_CMD_BATTERY                 0x65        // percentage of battery ( 0 to 100%, in 7.4 to 8.4V)
#define COM_CMD_RD_SW1                  0x66        // current condition of SSR1 ( on or off )
#define COM_CMD_WR_SW1                  0x67        // set condition of SSR1 ( on or off )
#define COM_CMD_RD_SW2                  0x68        // current condition of SSR2 ( on or off )
#define COM_CMD_WR_SW2                  0x69        // set condition of SSR2 ( on or off )



// Constant Value to calculate volume
#define COM_CMD_RD_C_BASE                 0x71  // read the area of tank        | in cm
#define COM_CMD_WR_C_BASE                 0x72  // write the area of tank       |
#define COM_CMD_RD_CAL                    0x73  // read calibration file ( refer to bsp.h )
#define COM_CMD_WR_CAL                    0x74  // write calibration file ( refer to bsp.h )

const byte cmd_list[]={
    COM_CMD_SLEEP,
    COM_CMD_RESTART,
    COM_CMD_PING,

//Manufacturing CMD
    COM_CMD_UUID,
    COM_CMD_MODEL,
    COM_CMD_FIRMWARE,
    COM_CMD_VERSION,
    COM_CMD_RD_SN,
    COM_CMD_WR_SN,
    COM_CMD_WR_HOST,

// Sensor & data flag
    COM_CMD_LEVEL,
    COM_CMD_VOLUME,
    COM_CMD_RAW_DATA,
    COM_CMD_SF,
    COM_CMD_BATTERY,
    COM_CMD_RD_SW1,
    COM_CMD_WR_SW1,
    COM_CMD_RD_SW2,
    COM_CMD_WR_SW2,

// Constant Value to calculate volume
    COM_CMD_RD_C_BASE,
    COM_CMD_WR_C_BASE,
    COM_CMD_RD_CAL,
    COM_CMD_WR_CAL
};


#endif