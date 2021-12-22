#ifndef _com_h_
#define _com_h_

#include "Arduino.h"
#include "RF24.h"
#include "pin_info.h"
#include "SPI.h"
#include "bsp.h"


/**
 *  CRC32 as checksum method, will be counted from Command till data. 
 * 
 */
/**
 * Data framing formation : 
 * - Header 
 * - Command
 * - Data length    // 0 ( zero ) if no data 
 * - Data ( 0 to n )
 * - CRC3       |
 * - CRC2       | using CRC32-bit, MSB first
 * - CRC1       |
 * - CRC0       |
 * - ACK/NACK  
 * - Footer     
 *      -> ACK/NACK is data-replied to server for status data 
 *      -> Footer is data received from server
 * 
 *  Special case on PING register
 *  a. If there is content ( data ) in PING package, the data must in 4bytes
 *  b. Data in PING package would be indicated as ID-host ( server ) where the module will reply to that ID
 *  c. Data replied in PING package would be containing Radio ID ( this module / ID client)
*/

/**
 * 
 *  Data position 
 * 
 *  Since the data will be cleared after HEADER_DATA 1 and 2 have been found 
 *  and  UART_START_Flag was true
 *  All data index will be reset to 0 ( zero ),  and starting to receive for the next data
 *  until found the END_DATA
 * */


/* Header */
#define COM_HEADER                      0xAA
#define COM_END                         0xFE

//Acknowledgement
#define COM_NAK                         0xC5
#define COM_ACK                         0x35

//Power CMD
#define COM_CMD_SLEEP                   0x78
#define COM_CMD_RESTART                 0x87
#define COM_CMD_PING                    0x89

//Manufacturing CMD
#define COM_CMD_UUID                    0x51
#define COM_CMD_MODEL                   0x52
#define COM_CMD_FIRMWARE                0x53
#define COM_CMD_VERSION                 0x54
#define COM_CMD_RN_SN                   0x55
#define COM_CMD_WR_SN                   0x56
#define COM_CMD_WR_HOST                 0x57

// Sensor & data flag
#define COM_CMD_LEVEL                   0x61        // water height or level in tank
#define COM_CMD_VOLUME                  0x62        // water volume in tank
#define COM_CMD_RAW_DATA                0x63        // raw data of adc
#define COM_CMD_SF                      0x64        // status of sensor
#define COM_CMD_RD_PUMP                 0x65        // current condition of pump ( on or off )
#define COM_CMD_WR_PUMP                 0x66        // set condition of pump ( on or off )

// Constant Value to calculate volume
#define COM_CMD_RD_C_HEIGHT               0x71
#define COM_CMD_WR_C_HEIGHT               0x72
#define COM_CMD_RD_C_BASE                 0x73
#define COM_CMD_WR_C_BASE                 0x74
#define COM_CMD_RD_CAL                    0x75
#define COM_CMD_WR_CAL                    0x76



#define RAW_CMD_LEN                     5

typedef enum 
{
    RADIO_OK    =   0,
    RADIO_ERROR =   1,
    RADIO_TIMEOUT=  2,
    RADIO_UNKNOWN=  3,
}RADIO_INFO;

typedef struct
{
    uint8_t header,
            cmd,
            data_length,
            nack_ack,
            footer,
            data_location,
            total_length;
    byte dataRaw[16];
    uint32_t CRC32_;
}CMD_Identifier;

class com
{
    protected : 
        static RF24 radio;
        static uint32_t id_radio;
        static bool isCMDValid(byte key);
        static bool isPackageValid(const uint8_t* raw, uint8_t length, CMD_Identifier* ret);

        static bool dataHandler(CMD_Identifier* var);
        // retrieve the length of data_package
        static uint8_t createPackage(uint8_t CMD, uint8_t nack_ack, uint8_t* raw, uint8_t len, uint8_t* package_, uint8_t package_len);
        static void clearRawCom(void);
    private :
        /**
         * byte numbers of data raw_com
         * 1. Header
         * 2. CMD
         * 3. DATA-length
         * 4. Location of data from raw-entries  
         * 5. Footer
         */
        static byte raw_com[RAW_CMD_LEN];    
        
        static CMD_Identifier raw_ident;
        static uint8_t radio_flag;

        // radio function transmission data
        static bool openTxRadio(uint32_t id_);
        static bool closeTxRadio(void);
        static bool openRxRadio(uint32_t id_);
        static bool closeRxRadio(void);

        static bool radioTransmit(uint32_t id_hos, const char* raw, uint8_t len);

    public :
        static RADIO_INFO initialize(void);
        static void loop(void);

};

#endif