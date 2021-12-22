#include "com.h"
#include <avr/pgmspace.h>
#include "pin_info.h"

RF24 com::radio;
uint32_t com::id_radio = 0;
uint8_t com::radio_flag = 0;

CMD_Identifier com::raw_ident = {0};

byte com::raw_com[RAW_CMD_LEN]={0}; 

// Clear the buffer of inital RAW
void com::clearRawCom(void)
{
    memset(raw_com,0,RAW_CMD_LEN);
}
const uint8_t key_word []=
{
    COM_CMD_SLEEP,
    COM_CMD_RESTART,
    COM_CMD_PING,
    COM_CMD_UUID,
    COM_CMD_MODEL,
    COM_CMD_FIRMWARE,
    COM_CMD_VERSION,
    COM_CMD_RN_SN,
    COM_CMD_WR_SN,
    COM_CMD_WR_HOST,
    COM_CMD_LEVEL,
    COM_CMD_VOLUME,
    COM_CMD_RAW_DATA,
    COM_CMD_SF,
    COM_CMD_RD_PUMP,
    COM_CMD_WR_PUMP,
    COM_CMD_RD_C_HEIGHT,
    COM_CMD_WR_C_HEIGHT,
    COM_CMD_WR_C_BASE,
    COM_CMD_RD_C_BASE,
    COM_CMD_RD_CAL,
    COM_CMD_WR_CAL,
};

bool com::isCMDValid(uint8_t key)
{
    for(uint8_t i=0; i<sizeof(key_word); i++)
    {
        uint8_t k = key_word[i];
        if(key==k)
            return true;
    }
    return false;
}

bool com::isPackageValid(const uint8_t* raw, uint8_t length, CMD_Identifier* ret)
{
    // memset((uint8_t*)ret, 0, sizeof(CMD_Identifier));
    uint8_t loc = 0;
    uint8_t proc = 0;
    int byte_left =0;
    uint32_t CRC_calc=0;
    Serial.println( F("data raw : "));
    while(loc<length)
    {
        Serial.print( (uint8_t)raw[loc] ); Serial.print(F("\t"));
        // printf("data raw : %d \r\n", raw[loc]);
        if( raw[loc] != 0x00  || loc < length) // compare with NULL
        {
            switch(proc)
            {
                case 0 :    // looking for header
                        if( raw[loc] == COM_HEADER )
                        {
                            // printf("found header\r\n");
                            ret->header=loc;
                            proc++;
                        }
                        break;
                case 1 :    // looking for key_word
                        if( com::isCMDValid(raw[loc]) )
                        {
                            // printf("found command\r\n");
                            ret->cmd=raw[loc];
                            proc++;
                        }
                        else
                            proc = 0;
                        break;
                case 2 :    // load the data_length
                        ret->data_length=raw[loc];
                        // printf("data length = %d\r\n", ret->data_length);
                        if( ret->data_length > 0 )
                        {
                            ret->data_location = loc+1;
                            byte_left = length - loc;
                            // printf("byte left is : %i", byte_left);
                            if(byte_left < (6+ret->data_length) )
                                return false;
                            else
                            {
                                if( ret->data_length > sizeof(ret->dataRaw))
                                    memcpy( ret->dataRaw, raw+ret->data_location, sizeof(ret->dataRaw) );
                                else
                                    memcpy( ret->dataRaw, raw+ret->data_location, ret->data_length );
                            };
                        }
                            
                        else
                            ret->data_location = 0;
                        proc++;
                        break;
                case 3 :    // load CRC-32bit
                        // confirming rest of byte
                        byte_left = length - loc;
                        if( byte_left < 6 )
                            return false;
                        if( ret->data_length > 0 )
                            loc +=ret->data_length;
                        ret->CRC32_ =  raw[loc];
                        ret->CRC32_ = (ret->CRC32_ << 8) | raw[loc+1];
                        ret->CRC32_ = (ret->CRC32_ << 8) | raw[loc+2];
                        ret->CRC32_ = (ret->CRC32_ << 8) | raw[loc+3];
                        CRC_calc = BSP::crc32((const char *)ret->dataRaw,ret->data_length);
                        if( CRC_calc != ret->CRC32_ )
                            return false;
                        proc++;
                        loc +=3;
                        // printf("data CRC = %x\r\n", ret->CRC32_);
                        // printf("data CRC from calculate : %x \r\n", CRC_calc);
                        
                        break;
                case 4 :    // confirming the footer
                default :
                        if( raw[loc+1] == COM_END )
                        {
                            ret->nack_ack = raw[loc];
                            // printf("found footer\r\n");
                            ret->footer=loc+1;
                            proc++;
                            return true;
                        }
                        else
                            proc = 0;
                        break;
            }    
        }
        else
            return false;
        if( proc > 4 )
            break;
        loc++;
    }
    if( proc > 4)
        return true;
    if( loc >= length && proc <= 4 )
        return false;
    return true;
}

bool com::dataHandler(CMD_Identifier* var)
{
    uint8_t bytes_tmp[4];
    bool need_send_data = false;
    uint32_t tmp_id_host = BSP::getHOST_ID();
    uint8_t nack_ack_state = COM_ACK;
    uint8_t ptr_len = 0;
    uint8_t *ptr = (uint8_t*) malloc(32 * sizeof(uint8_t));
    if ( ptr == NULL )
        return false;
    if(!com::isCMDValid(var->cmd))
        return false;
    switch (var->cmd)
    {
        case COM_CMD_SLEEP :
        case COM_CMD_RESTART:
            BSP::resetFunc();
            break;
        case COM_CMD_PING :
            if( var->data_length > 0 )
            {
                if( var->data_length == sizeof(uint32_t) )
                {
                    BSP::fourBytesToUint32(var->dataRaw, &tmp_id_host);
                    nack_ack_state = COM_ACK;
                }
                else
                    nack_ack_state = COM_NAK;
            }
            BSP::uint32To4bytes(com::id_radio, bytes_tmp);
            ptr_len = com::createPackage(var->cmd, nack_ack_state, bytes_tmp, 4, ptr, 30);
            need_send_data = true;
            break;
        case COM_CMD_UUID :
            ptr_len = com::createPackage(var->cmd, nack_ack_state, (uint8_t*) BSP::getINFO_UUID, 10, ptr, 30);
            need_send_data = true;
            break;
        case COM_CMD_MODEL :
            ptr_len = com::createPackage(var->cmd, nack_ack_state, (uint8_t*) BSP::getINFO_MODEL, 12, ptr, 30);
            need_send_data = true;
        case COM_CMD_FIRMWARE :
            ptr_len = com::createPackage(var->cmd, nack_ack_state, (uint8_t*) BSP::getINFO_firmware, 12, ptr, 30);
            need_send_data = true;
        case COM_CMD_VERSION :
            ptr_len = com::createPackage(var->cmd, nack_ack_state, (uint8_t*) BSP::getINFO_VERSION, 12, ptr, 30);
            need_send_data = true;
        case COM_CMD_RN_SN :
            ptr_len = com::createPackage(var->cmd, nack_ack_state, (uint8_t*) BSP::getINFO_SN, 24, ptr, 30);
            need_send_data = true;
        case COM_CMD_WR_SN :
            if( !BSP::updateBOARD_INFO(DEV_SN, reinterpret_cast<char*>(var->dataRaw), var->data_length) )
                nack_ack_state = COM_NAK;
            else
                nack_ack_state = COM_ACK;
            ptr_len = com::createPackage(var->cmd, nack_ack_state, (uint8_t*) BSP::getINFO_SN, 24, ptr, 30);
            need_send_data = true;
            break;
        case COM_CMD_WR_HOST :
            if( !BSP::updateBOARD_INFO(DEV_SN, reinterpret_cast<char*>(var->dataRaw), var->data_length) )
                nack_ack_state = COM_NAK;
            else
                nack_ack_state = COM_ACK;
            tmp_id_host = BSP::getHOST_ID();
            BSP::uint32To4bytes(tmp_id_host, bytes_tmp);
            ptr_len = com::createPackage(var->cmd, nack_ack_state, bytes_tmp, 4, ptr, 30);
            need_send_data = true;
            break;
        case COM_CMD_LEVEL :
            BSP::floatToFourByte(bytes_tmp, BSP::getHeight());
            nack_ack_state  = var->nack_ack;
            ptr_len = com::createPackage(var->cmd, nack_ack_state, bytes_tmp, 4, ptr, 30);
            need_send_data = true;
            break;
        case COM_CMD_VOLUME :
            BSP::floatToFourByte(bytes_tmp, BSP::getVolume());
            nack_ack_state  = var->nack_ack;
            ptr_len = com::createPackage(var->cmd, nack_ack_state, bytes_tmp, 4, ptr, 30);
            need_send_data = true;
            break;
        case COM_CMD_RAW_DATA :
            BSP::uint32To4bytes(BSP::getSensorRAW(), bytes_tmp);
            nack_ack_state  = var->nack_ack;
            ptr_len = com::createPackage(var->cmd, nack_ack_state, bytes_tmp, 4, ptr, 30);
            need_send_data = true;
            break;
        case COM_CMD_SF :
            nack_ack_state  = var->nack_ack;
            bytes_tmp[0] = BSP::getSensorStatus();
            ptr_len = com::createPackage(var->cmd, nack_ack_state, &bytes_tmp[0], 1, ptr, 30);
            need_send_data = true;
            break;
        case COM_CMD_RD_PUMP :
            nack_ack_state  = var->nack_ack;
            bytes_tmp[0] = BSP::getMotorStatus();
            ptr_len = com::createPackage(var->cmd, nack_ack_state, &bytes_tmp[0], 1, ptr, 30);
            need_send_data = true;
            break;
        case COM_CMD_WR_PUMP:
            nack_ack_state  = var->nack_ack;
            BSP::setMotorStatus( ( var->dataRaw[0]> 0 ? HIGH : LOW ) );
            bytes_tmp[0] = BSP::getMotorStatus();
            ptr_len = com::createPackage(var->cmd, nack_ack_state, &bytes_tmp[0], 1, ptr, 30);
            need_send_data = true;
            break;
        case COM_CMD_RD_C_HEIGHT :
            BSP::floatToFourByte(bytes_tmp, BSP::getSensorConstHeigth());
            nack_ack_state  = var->nack_ack;
            ptr_len = com::createPackage(var->cmd, nack_ack_state, bytes_tmp, 4, ptr, 30);
            need_send_data = true;
            break;
        case COM_CMD_WR_C_HEIGHT :
            break;
        case COM_CMD_RD_C_BASE :
            BSP::floatToFourByte(bytes_tmp, BSP::getSensorConstBase());
            nack_ack_state  = var->nack_ack;
            ptr_len = com::createPackage(var->cmd, nack_ack_state, bytes_tmp, 4, ptr, 30);
            need_send_data = true;
            break;
        case COM_CMD_WR_C_BASE :
            break;
        case COM_CMD_RD_CAL:
            nack_ack_state  = var->nack_ack;
            CAL_PARAM tmp_cal = BSP::getSENSOR_CAL();
            ptr_len = com::createPackage(var->cmd, nack_ack_state, reinterpret_cast<uint8_t*>(&tmp_cal), sizeof(CAL_PARAM), ptr, 30);
            need_send_data = true;
            break;
        case COM_CMD_WR_CAL:
            break;
        default:
            return false;
            break;
    };
    
    if( need_send_data )
    {
        com::radio.stopListening();
        com::radio.openWritingPipe((uint64_t)tmp_id_host);
        if( !com::radio.write(ptr, ptr_len) )
        {
            delay(100);
            need_send_data = com::radio.write(ptr, ptr_len);
        }
        else
            need_send_data = true;
        radio.openReadingPipe(1,com::id_radio);
        radio.startListening();
    }
    free(ptr);
    return need_send_data;
}


/**
 * @brief 
 * 
 * 
 * 
 * @param CMD           refer to CMD table
 * @param nack_ack      status NACK or ACK
 * @param raw           data raw, plain data would be encapsulated to package-data
 * @param len           length of data raw
 * @param package_      data array ( packet-data ), to be sent from NRF
 * @param package_len   length of packet-data
 * @return uint8_t 
 * 
 * 
 * how to use 
 * float data_send = 0.345;
    uint8_t data_raw[32];
    uint8_t len_raw = 32;
    memset(data_raw, 0, 32);
    uint8_t databytes[sizeof(data_send)];
    floatToFourByte(databytes,data_send);
    for(uint8_t lo=0; lo<sizeof(data_send); lo++)
    {
        printf("\t %d", databytes[lo]);
    }
    
    if( createPackage(COM_CMD_VOLUME, COM_ACK, &databytes, sizeof(databytes), data_raw, len_raw) >=9 )
    {
        printf("\r\ndata in bytes : %i", (uint32_t)data_send);
        for(uint8_t l=0; l< sizeof(data_raw); l++)
        {
            printf ( " \t %d", data_raw[l]);
        }
    }
    float data_send_retrieve = 0;
    fourByteToFloat(databytes, &data_send_retrieve);
    printf("\r\n retrieve %f", data_send_retrieve);
 */
uint8_t com::createPackage(uint8_t CMD, uint8_t nack_ack, uint8_t* raw, uint8_t len, uint8_t* package_, uint8_t package_len)
{
    if( !isCMDValid(CMD) )
        return false;
    if ( nack_ack != COM_ACK && nack_ack != COM_NAK )
        return 0;
    if( package_len > 32 )
        package_len = 32;
    if( package_len < 9 ) // minimum data size, encapsulated size in bytes is 9
        return 0;
    if( len > 23 )      // max payload of NRF is 32bytes, but 9 bytes already used by package encapsulated
        return 0;
    // create header
    package_[0] = COM_HEADER;
    // command
    package_[1] = CMD;
    // data_length
    package_[2] = len;
    // data raw
    if( len > 0 )
    {
        for(uint8_t l = 0; l < len; l++)
        {
            if ( 3+l <= 32 )
                package_[3+l] = raw[l];
        }
        
    }
    // CRC32-bit
    uint32_t CRC32_ = BSP::crc32((const char*)raw,len);
    uint8_t crcByte[4];
    BSP::uint32To4bytes(CRC32_, crcByte);
    for(uint8_t l=0; l<4; l++)
    {
        package_[3+len+l] = crcByte[0];    
    }
    // NACK/ACK
    package_[3+len+4] = nack_ack;
    // Footer
    package_[3+len+5] = COM_END;
    return ( 9+len );
    // printf(" length of data is : %d\r\n", 3+len+5);
}


RADIO_INFO com::initialize(void)
{
    char UUID_[MAX_UUID_LENGTH];
    BSP::BOARD_GET_UUID(UUID_,MAX_UUID_LENGTH);
    Serial.print(F("UUID radio: "));
    for( uint8_t ui = 0; ui < MAX_UUID_LENGTH; ui++)
    {
        Serial.print(F("0x"));
        Serial.print(UUID_[ui],HEX);
        Serial.print(F("\t"));
    }
    com::id_radio = (uint32_t)str2uint64(UUID_);
    SPI.begin();
    if (!radio.begin(&SPI, CE_PIN,CSN_PIN)) 
        radio_flag = (uint8_t) RADIO_ERROR; 
    else
    {
        radio_flag = (uint8_t) RADIO_OK;
        Serial.print("open id radio ");
        uint32_t lsb = com::id_radio>>8 & 0xffffffff;
        uint32_t msb = com::id_radio & 0xffffffff;
        Serial.print(lsb, HEX);
        Serial.println(msb, HEX);
        radio.openReadingPipe(1,(uint64_t)com::id_radio);
        radio.startListening();
    }
    return (RADIO_INFO)radio_flag;       
}

void com::loop(void)
{
    if (radio.available())
    { //jika terbaca data di module
        Serial.println("message :");
        bool selesai = false;
        char buffer[32];
        while (!selesai)
        { 
            radio.read(buffer, 32); 
            Serial.println(buffer); 
            delay(10);
            selesai = !radio.available();
        }
        // memset(&raw_ident, sizeof(CMD_Identifier), 0);
        raw_ident={0};
        if( com::isPackageValid((const uint8_t*)buffer, sizeof(buffer), &raw_ident) )
        {
            com::dataHandler(&raw_ident);
        }
    }   
    else
    {
        Serial.println("No radio available");
    }
}