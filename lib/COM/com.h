#ifndef _com_h_
#define _com_h_

#include "Arduino.h"
#include "RF24.h"
#include "pin_info.h"
#include "SPI.h"
#include "bsp.h"
#include "cmdTable.h"
#include <serialProtocol.h>


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


#define RAW_CMD_LEN                     5

typedef enum 
{
    RADIO_OK    =   0,
    RADIO_ERROR =   1,
    RADIO_TIMEOUT=  2,
    RADIO_UNKNOWN=  3,
}RADIO_INFO;


class com
{
    protected : 
        static RF24 radio;
        static uint16_t id_radio;
        static uint64_t id_dest;
        static uint8_t rawData[32];
        static uint8_t rawLen;
        static uint8_t radioFlag;
        static uint32_t timeInMillis;
    private :       
        static CMD_Identifier raw_ident;
        static uint8_t radio_flag;

        // radio function transmission data
        static bool radioTransmit(uint64_t id_host, const char* raw, uint8_t len, uint8_t tried);

        static serialProtocol cmdProtocol;
        static protocolPtr funct[TOTAL_CMD];

        static void restartISR(void);
        static void pingISR(void);

        // Interrupt sub-routine vector for ACK/NACK protocol
        // classify in 4 
        static void powerISR(void);
        static void manufacturingISR(void);
        static void sensorPullISR(void);
        static void sensorPushISR(void);
    public :
        static RADIO_INFO initialize(void);
        static void loop(void);

        static void setDest_ID(uint64_t id); //{ com::id_dest = id; }
        static void setSrc_ID(uint64_t id);//  { com::id_radio = id;}

};

#endif