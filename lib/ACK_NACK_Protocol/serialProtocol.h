#ifndef SERIALPROTOCOL_H_
#define SERIALPROTOCOL_H_


#include "Arduino.h"
#include "stdint.h"
#include "string.h"

/**
 *  CRC32 as checksum method, counting data only. 
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
#define COM_UNKNWON                     0xFF

//Acknowledgement
#define COM_NAK                         0xC5
#define COM_ACK                         0x35

#ifndef SERIAL_P
    #define SERIAL_P    Serial
#endif

#ifndef MAX_RAW_LENGTH   
    #define MAX_RAW_LENGTH  24
#endif

typedef struct
{
    uint8_t header,
            cmd,
            data_length,
            nack_ack,
            footer,
            data_location,
            total_length;
    byte dataRaw[MAX_RAW_LENGTH];
    uint32_t CRC32_;
}CMD_Identifier;
typedef void (*protocolPtr)(void);

class serialProtocol
{
    private :
        CMD_Identifier local_ident;
        byte* keyWord;
        protocolPtr *ptr_;
        uint8_t key_len = 0;
        uint8_t ptr_len = 0;
        uint32_t crc32(const char *s,size_t n) 
        {
            uint32_t crc=0xFFFFFFFF;
            
            for(size_t i=0;i<n;i++) {
                char ch=s[i];
                for(size_t j=0;j<8;j++) {
                    uint32_t b=(ch^crc)&1;
                    crc>>=1;
                    if(b) crc=crc^0xEDB88320;
                    ch>>=1;
                }
            }
            return ~crc;
        };
        void uint32To4bytes(uint32_t res, uint8_t* dest)
        {
            // MSB first
            memset(dest,'\0',4);
            dest[0] = ( res >> 24 ) & 0xff;
            dest[1] = ( res >> 16 ) & 0xff;
            dest[2] = ( res >> 8 )  & 0xff;
            dest[3] = res & 0xff;
        };
        bool isCMDValid(byte key);
        bool isPackageValid(const uint8_t* raw, uint8_t length, CMD_Identifier* ret);
    public :
        serialProtocol();
        serialProtocol(byte* key, uint8_t len);
        serialProtocol(byte* key, uint8_t len, protocolPtr *ptr_list, uint8_t ptr_len);
        ~serialProtocol();
        
        // attach your key list
        void attachKey(byte* key, uint8_t len);

        void attachISR(protocolPtr *ptr_list, uint8_t ptr_len);
        // retrieve the length of data_package
        uint8_t createPackage(byte CMD, uint8_t nack_ack, uint8_t* raw, uint8_t len, uint8_t* package_, uint8_t package_len);

        // update screening data
        bool updatePackage(const char* dataRaw, uint8_t len);

        template <typename T, size_t size_>
        bool updatePackage(T(&series)[size_])
        {
            return updatePackage(series,size_);
        }
        
        // return of variable
        uint8_t getCurrentCMD(void)             { return this->local_ident.cmd; }
        uint8_t getDataLength(void)             { return this->local_ident.data_length; }
        CMD_Identifier getCMDIdentifier(void)   { return this->local_ident; }

};
#endif