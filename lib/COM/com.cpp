#include "com.h"
#include <string.h>
#include "cmdTable.h"
#include "SoftwareSerial.h"
#include <avr/pgmspace.h>
#include "pin_info.h"

RF24 com::radio(CE_NRF, CSN_NRF);
uint16_t com::id_radio = 0;
uint64_t com::id_dest  = 123; // for testing only
uint8_t com::radio_flag = 0;

uint8_t com::rawData[32];
uint8_t com::rawLen = 0;


uint8_t com::radioFlag = LOW;
uint32_t com::timeInMillis = 0;

CMD_Identifier com::raw_ident = {0};
protocolPtr com::funct[TOTAL_CMD]=
                                    {
                                        // PWR cmd
                                        NULL,                               // for sleep cmd
                                        &com::powerISR,//&com::restartISR,                   // for restart cmd
                                        &com::powerISR,//&com::pingISR,                      // for ping cmd 

                                        // manufacture CMD
                                        &com::manufacturingISR,             // UUID
                                        &com::manufacturingISR,             // MODEL
                                        &com::manufacturingISR,             // FIRMWARE
                                        &com::manufacturingISR,             // VERSION
                                        &com::manufacturingISR,             // READ_serial NUMBER
                                        &com::manufacturingISR,             // WRITE serial Number
                                        &com::manufacturingISR,             // Write HOST id

                                        // sensor PULL
                                        &com::sensorPullISR,                // water level
                                        &com::sensorPullISR,                // water volume
                                        &com::sensorPullISR,                // raw data of sensor water
                                        &com::sensorPullISR,                // sensor status
                                        &com::sensorPullISR,                // battery percent
                                        &com::sensorPullISR,                // read SW1
                                        &com::sensorPullISR,                // Write SW1
                                        &com::sensorPullISR,                // read SW2
                                        &com::sensorPullISR,                // write SW2

                                        // sensor PUSH data
                                        &com::sensorPushISR,
                                        &com::sensorPushISR,
                                        &com::sensorPushISR,
                                        &com::sensorPushISR,
                                    };

serialProtocol com::cmdProtocol(cmd_list,sizeof(cmd_list), com::funct, TOTAL_CMD);

extern SoftwareSerial mySer;

void com::restartISR(void)
{
    BSP::resetFunc();
}

void com::pingISR(void)
{
    //mySer.println("get ping data !!");
    uint8_t buffRaw[32];
    uint8_t buffLen=0;
    com::raw_ident=com::cmdProtocol.getCMDIdentifier();
    buffLen = com::cmdProtocol.createPackage(COM_CMD_PING,com::raw_ident.nack_ack,(uint8_t*)&com::id_radio,sizeof(com::id_radio), buffRaw, 32);
    //mySer.println("data reply (raw) : ");
    for(uint8_t ui=0; ui<buffLen; ui++)
    {
        //mySer.print(buffRaw[ui],DEC);
        //mySer.print("  ");
    }
    if( buffLen > 0)    
    {
        com::radio.stopListening();
        if(com::raw_ident.data_length > 1 && com::raw_ident.data_length <= 2)       // exactly for 16 bit of ID
        {
            uint16_t id_dest = com::raw_ident.dataRaw[0] |  (com::raw_ident.dataRaw[1]<<8);
            //mySer.println("destination : "+String(id_dest,DEC));
            com::radio.openWritingPipe((uint64_t)id_dest);
        }
        bool report = com::radio.write(buffRaw,buffLen);
        if( !report )
        {
            for( uint8_t tr=0; tr<5; tr++)
            {
                delay(100);
                report = com::radio.write(buffRaw,buffLen);
                //mySer.println("retry to send data");
                if(report)
                    break;
            }
            
            
        }else
        {
            //mySer.println("success");
        }
        // com::radio.openWritingPipe((uint64_t)BSP::getHOST_ID());
        com::radio.startListening();
    }else
    {
        // NOPE
    }
    
}

void com::powerISR(void)
{
    uint64_t id_tp=0;
    memset(com::rawData,0,32);
    com::rawLen = 0;
    com::raw_ident=com::cmdProtocol.getCMDIdentifier();
    switch(com::raw_ident.cmd)
    {
        case COM_CMD_SLEEP:
                            break;
        case COM_CMD_RESTART:
                            BSP::resetFunc();
                            break;
        case COM_CMD_PING:
                            // create data raw
                            com::rawLen = com::cmdProtocol.createPackage(COM_CMD_PING,com::raw_ident.nack_ack,(uint8_t*)&com::id_radio,sizeof(com::id_radio), com::rawData, 32);
                            if( com::rawLen > 0 )   // perventive action, so there is no dumb or pseudo raw data will be sent
                            {
                                // create ID host if there is ID in the packets
                                if(com::raw_ident.data_length > 0)
                                {
                                    uint8_t loc_data = 0;
                                    for ( uint8_t idx = 0; idx < com::raw_ident.data_length; idx++)
                                    {
                                        loc_data = com::raw_ident.data_length - 1 - idx;
                                        id_tp = id_tp << 8;
                                        id_tp |= com::raw_ident.dataRaw[loc_data];
                                    }
                                    //mySer.print(" get ping data from : ");
                                    //mySer.println( (uint32_t)id_tp );
                                    com::radioTransmit(id_tp, (const char*)com::rawData, com::rawLen, 5);
                                }else
                                {
                                    com::radioTransmit(com::id_dest, (const char*) com::rawData, com::rawLen, 5);
                                }
                            }
                            break;
    };
    com::raw_ident={0};
}

void com::manufacturingISR(void)
{
    uint64_t id_tp=0;
    memset(com::rawData,0,32);
    com::rawLen = 0;
    com::raw_ident=com::cmdProtocol.getCMDIdentifier();
    bool changeID=false;
    uint8_t strLen_=0;
    switch(com::raw_ident.cmd)
    {
        case COM_CMD_UUID :
                                strLen_ = (uint8_t)strlen(BSP::getINFO_UUID());
                                com::rawLen = com::cmdProtocol.createPackage(COM_CMD_UUID,com::raw_ident.nack_ack,(uint8_t*)BSP::getINFO_UUID(),strLen_, com::rawData, 32);
                                //mySer.println(BSP::getINFO_UUID());
                                break;
        case COM_CMD_MODEL :
                                strLen_ = strlen(BSP::getINFO_MODEL());
                                if( strLen_ < 1)
                                {
                                    strLen_ = strlen(DEFAULT_MODEL);
                                    com::rawLen = com::cmdProtocol.createPackage(COM_CMD_MODEL,com::raw_ident.nack_ack,(const char*)DEFAULT_MODEL,strLen_, com::rawData, 32);
                                }
                                else
                                    com::rawLen = com::cmdProtocol.createPackage(COM_CMD_MODEL,com::raw_ident.nack_ack,BSP::getINFO_MODEL(),strLen_, com::rawData, 32);
                                break;
        case COM_CMD_FIRMWARE :
                                strLen_ = strlen(BSP::getINFO_firmware());
                                com::rawLen = com::cmdProtocol.createPackage(COM_CMD_FIRMWARE,com::raw_ident.nack_ack,(uint8_t*)BSP::getINFO_firmware(),strLen_, com::rawData, 32);
                                break;
        case COM_CMD_VERSION :
                                strLen_ = strlen(BSP::getINFO_VERSION());
                                com::rawLen = com::cmdProtocol.createPackage(COM_CMD_VERSION,com::raw_ident.nack_ack,(uint8_t*)BSP::getINFO_VERSION(),strLen_, com::rawData, 32);
                                break;
        case COM_CMD_RD_SN :
                                strLen_ = strlen(BSP::getINFO_SN());
                                com::rawLen = com::cmdProtocol.createPackage(COM_CMD_RD_SN,com::raw_ident.nack_ack,(uint8_t*)BSP::getINFO_SN(),strLen_, com::rawData, 32);
                                break;
        case COM_CMD_WR_SN :
                                // BSP::
                                BSP::setINFO_SN( (const char*) com::raw_ident.dataRaw);
                                strLen_ = strlen(BSP::getINFO_SN());
                                com::rawLen = com::cmdProtocol.createPackage(COM_CMD_WR_SN,com::raw_ident.nack_ack,(uint8_t*)BSP::getINFO_SN(),strLen_, com::rawData, 32);
                                break;
        case COM_CMD_WR_HOST :
                                // create ID host if there is ID in the packets
                                if(com::raw_ident.data_length > 0)
                                {
                                    uint8_t loc_data = 0;
                                    for ( uint8_t idx = 0; idx < com::raw_ident.data_length; idx++)
                                    {
                                        loc_data = com::raw_ident.data_length - 1 - idx;
                                        id_tp = id_tp << 8;
                                        id_tp |= com::raw_ident.dataRaw[loc_data];
                                    }
                                    BSP::setHOST_ID((uint32_t)id_tp);
                                    com::id_dest = (uint64_t)BSP::getHOST_ID;
                                    strLen_ = sizeof(com::id_dest);
                                    com::rawLen = com::cmdProtocol.createPackage(COM_CMD_WR_HOST,com::raw_ident.nack_ack,(uint8_t*)&com::id_dest,strLen_, com::rawData, 32);
                                    //mySer.print(" Write host to : ");
                                    //mySer.println( (uint32_t)id_tp );
                                }else
                                {
                                    com::rawLen = com::cmdProtocol.createPackage(COM_CMD_WR_HOST,COM_NAK,NULL,0, com::rawData, 32);
                                }
                                break;
    };
    
    

    if( com::rawLen > 0 )
    {
            // create ID host if there is ID in the packets
            if(com::raw_ident.data_length > 0 && ( com::raw_ident.cmd != COM_CMD_WR_SN ))
            {
                uint8_t loc_data = 0;
                for ( uint8_t idx = 0; idx < com::raw_ident.data_length; idx++)
                {
                    loc_data = com::raw_ident.data_length - 1 - idx;
                    id_tp = id_tp << 8;
                    id_tp |= com::raw_ident.dataRaw[loc_data];
                    if( idx >= 8)
                        break;
                }
                com::radioTransmit(id_tp, (const char*)com::rawData, com::rawLen, 5);
            }else
            {
                com::radioTransmit(com::id_dest, (const char*) com::rawData, com::rawLen, 5);
            }
    }
        // com::radioTransmit(com::id_dest, (const char*) com::rawData, com::rawLen, 5);
}


void com::sensorPullISR(void)
{
    memset(com::rawData,0,32);
    com::rawLen = 0;
    com::raw_ident  = com::cmdProtocol.getCMDIdentifier();
    uint8_t strLen_ = 0;
    union{
            float f;
            uint32_t u;
            uint8_t bytes_[4];
    }tmp;
    switch(com::raw_ident.cmd)
    {
        case COM_CMD_LEVEL:             // load water level in tank
                            tmp.f = BSP::getHeight();
                            strLen_ = sizeof(tmp.f);
                            com::rawLen = com::cmdProtocol.createPackage(COM_CMD_LEVEL,com::raw_ident.nack_ack,(uint8_t*)&tmp.f,strLen_, com::rawData, 32);
                            break;  
        case COM_CMD_VOLUME:            // load volume of water in tank ( by calculation )
                            tmp.f = BSP::getVolume();
                            strLen_ = sizeof(tmp.f);
                            com::rawLen = com::cmdProtocol.createPackage(COM_CMD_VOLUME,com::raw_ident.nack_ack,(uint8_t*)&tmp.f,strLen_, com::rawData, 32);
                            break;
        case COM_CMD_RAW_DATA:          // load the raw data of water-level sensor in 8bytes
                            tmp.u = BSP::getSensorRAW();
                            strLen_ = sizeof(tmp.u);
                            com::rawLen = com::cmdProtocol.createPackage(COM_CMD_RAW_DATA,com::raw_ident.nack_ack,(uint8_t*)&tmp.u,strLen_, com::rawData, 32);
                            break;
        case COM_CMD_SF:                // load the status of sensor, available or not
                            tmp.bytes_[0] = BSP::getSensorStatus();
                            strLen_ = sizeof(tmp.bytes_[0]);
                            com::rawLen = com::cmdProtocol.createPackage(COM_CMD_SF,com::raw_ident.nack_ack,(uint8_t*)&tmp.bytes_[0],strLen_, com::rawData, 32);
                            break;
        case COM_CMD_BATTERY:           // load the percentage of battery ( 0 - 100% ), floating-number
                            tmp.f = BSP::getBatteryPercent();
                            strLen_ = sizeof(tmp.f);
                            com::rawLen = com::cmdProtocol.createPackage(COM_CMD_BATTERY,com::raw_ident.nack_ack,(uint8_t*)&tmp.f,strLen_, com::rawData, 32);
                            break;
        case COM_CMD_RD_SW1:            // read status of SWITCH 1
                            tmp.bytes_[0] = BSP::getSW1Status();
                            strLen_ = sizeof(tmp.bytes_[0]);
                            com::rawLen = com::cmdProtocol.createPackage(COM_CMD_RD_SW1,com::raw_ident.nack_ack,(uint8_t*)&tmp.bytes_[0],strLen_, com::rawData, 32);
                            break;
        case COM_CMD_WR_SW1:            // set the SWITCH1 to be closed of opened
                            if(com::raw_ident.data_length == 1)
                            {
                                tmp.bytes_[0]=com::raw_ident.dataRaw[0];
                                BSP::setSW1Status(tmp.bytes_[0]);
                                strLen_ = sizeof(tmp.bytes_[0]);
                            }else
                            {
                                tmp.bytes_[0]=BSP::getSW1Status();
                                strLen_ = sizeof(tmp.bytes_[0]);
                                com::raw_ident.nack_ack=COM_NAK;
                            }
                            com::rawLen = com::cmdProtocol.createPackage(COM_CMD_WR_SW1,com::raw_ident.nack_ack,(uint8_t*)&tmp.bytes_[0],strLen_, com::rawData, 32);
                            break;  
        case COM_CMD_RD_SW2:            // read status of SWITCH 2
                            tmp.bytes_[0] = BSP::getSW2Status();
                            strLen_ = sizeof(tmp.bytes_[0]);
                            com::rawLen = com::cmdProtocol.createPackage(COM_CMD_RD_SW2,com::raw_ident.nack_ack,(uint8_t*)&tmp.bytes_[0],strLen_, com::rawData, 32);
                            break;
        case COM_CMD_WR_SW2:            // set the SWITCH2 to be closed or opened
                            if(com::raw_ident.data_length == 1)
                            {
                                tmp.bytes_[0]=com::raw_ident.dataRaw[0];
                                BSP::setSW2Status(tmp.bytes_[0]);
                                strLen_ = sizeof(tmp.bytes_[0]);
                            }else
                            {
                                tmp.bytes_[0]=BSP::getSW2Status();
                                strLen_ = sizeof(tmp.bytes_[0]);
                                com::raw_ident.nack_ack=COM_NAK;
                            }
                            com::rawLen = com::cmdProtocol.createPackage(COM_CMD_WR_SW2,com::raw_ident.nack_ack,(uint8_t*)&tmp.bytes_[0],strLen_, com::rawData, 32);
                            break;
        default :
                return;     // quit from sub-routine if there is no Code
                break;  // actually it wouldn't be executed, but this is preventive action only
    };
    uint64_t id_tp;
    if( com::rawLen > 0 )
    {
            // create ID host if there is ID in the packets
            if(com::raw_ident.data_length > 0 && ( (com::raw_ident.cmd != COM_CMD_WR_SW2) && (com::raw_ident.cmd != COM_CMD_WR_SW1) ) )
            {
                uint8_t loc_data = 0;
                for ( uint8_t idx = 0; idx < com::raw_ident.data_length; idx++)
                {
                    loc_data = com::raw_ident.data_length - 1 - idx;
                    id_tp = id_tp << 8;
                    id_tp |= com::raw_ident.dataRaw[loc_data];
                    if( idx >= 8)
                        break;
                }
                com::radioTransmit(id_tp, (const char*)com::rawData, com::rawLen, 5);
            }else
            {
                com::radioTransmit(com::id_dest, (const char*) com::rawData, com::rawLen, 5);
            }
    }
        // com::radioTransmit(com::id_dest, (const char*) com::rawData, com::rawLen, 5);
}

void com::sensorPushISR(void)
{
    memset(com::rawData,0,32);
    com::rawLen = 0;
    com::raw_ident  = com::cmdProtocol.getCMDIdentifier();
    uint8_t strLen_ = 0;
    union{
            float f;
            uint32_t u;
            uint8_t bytes_[4];
    }tmp;
    CAL_PARAM cal_file={0};
    switch(com::raw_ident.cmd)
    {
        case COM_CMD_RD_C_BASE:
                                tmp.f = BSP::getSensorConstBase();
                                strLen_ = sizeof(tmp.f);
                                com::rawLen = com::cmdProtocol.createPackage(COM_CMD_RD_C_BASE,com::raw_ident.nack_ack,(uint8_t*)&tmp.f,strLen_, com::rawData, 32);
                                break;
        case COM_CMD_WR_C_BASE:
                                if( com::raw_ident.data_length == sizeof(float) )
                                {
                                    for( uint8_t lo = 0; lo < 4; lo++)
                                    {
                                        tmp.bytes_[lo]=com::raw_ident.dataRaw[lo];
                                    }
                                    if( tmp.f > 0 )
                                    {
                                        BSP::setSensorConstBase(tmp.f);
                                        strLen_ = sizeof(tmp.f);
                                        com::rawLen = com::cmdProtocol.createPackage(COM_CMD_WR_C_BASE,com::raw_ident.nack_ack,(uint8_t*)&tmp.f,strLen_, com::rawData, 32);
                                    }
                                    else
                                    {
                                        com::rawLen = com::cmdProtocol.createPackage(COM_CMD_WR_C_BASE,COM_NAK,NULL,0, com::rawData, 32);
                                    }
                                }
                                else
                                    com::rawLen = com::cmdProtocol.createPackage(COM_CMD_WR_C_BASE,COM_NAK,NULL,0, com::rawData, 32);
                                break;
        case COM_CMD_RD_CAL:
                                cal_file = BSP::getSENSOR_CAL();
                                strLen_ = sizeof(cal_file);
                                com::rawLen = com::cmdProtocol.createPackage(COM_CMD_RD_CAL,com::raw_ident.nack_ack,(uint8_t*)&cal_file,strLen_, com::rawData, 32);
                                break;
        case COM_CMD_WR_CAL:
                                if( com::raw_ident.data_length == sizeof(CAL_PARAM) )
                                {
                                    uint8_t *raw = (uint8_t*)&cal_file;
                                    for( uint8_t lo = 0; lo < sizeof(CAL_PARAM); lo++)
                                    {
                                        *raw = com::raw_ident.dataRaw[lo];
                                        raw++;
                                    }
                                    if( !isnanf((double)cal_file.gain) &&  !isnanf((double)cal_file.offset) )
                                    {
                                        BSP::updateSENSOR_CAL(&cal_file);
                                        strLen_ = sizeof(cal_file);
                                        com::rawLen = com::cmdProtocol.createPackage(COM_CMD_WR_CAL,com::raw_ident.nack_ack,(uint8_t*)&cal_file,strLen_, com::rawData, 32);
                                    }
                                    else
                                    {
                                        com::rawLen = com::cmdProtocol.createPackage(COM_CMD_WR_CAL,COM_NAK,NULL,0, com::rawData, 32);
                                    }
                                }
                                else
                                    com::rawLen = com::cmdProtocol.createPackage(COM_CMD_WR_CAL,COM_NAK,NULL,0, com::rawData, 32);
                                break;
    };
    if( com::rawLen > 0 )
        com::radioTransmit(com::id_dest, (const char*) com::rawData, 32/*com::rawLen*/, 5);
}

bool com::radioTransmit(uint64_t id_host, const char* raw, uint8_t len, uint8_t tried)
{
    // for(uint8_t i = 0; i < len; i++)
    // {
    //   mySer.print(" \t 0x");
    //   mySer.print(raw[i],HEX);
    // }    
    // mySer.println("send data to "+String((uint32_t)id_host));
    bool report = false;
    com::radio.stopListening();
    delay(5);
    if( id_host != com::id_dest )
    {
        com::radio.openWritingPipe(id_host);
        delay(10);
    }
    report = com::radio.write(raw,len);
    if( true )
    {
        for( uint8_t tr=0;  tr < tried; tr++)
        {
            delay(100);
            report = com::radio.write(raw,len);
            if(report)
                break;
        }   
    }
    if( id_host != com::id_dest )
    {
        com::radio.openWritingPipe(com::id_dest);
        delay(50);
    }
    com::radio.flush_tx();
    com::radio.flush_rx();
    com::radio.startListening();
    return report;
}


void com::setDest_ID(uint64_t id) 
{ 
    if( id == com::id_dest )
        return;

    com::id_dest = id; 
    com::radio.stopListening();
    com::radio.openWritingPipe(com::id_dest);
    com::radio.flush_tx();
    com::radio.startListening();

}

void com::setSrc_ID(uint64_t id)  
{ 
    if( id == com::id_radio )
        return;

    com::id_radio = id;
    com::radio.stopListening();
    com::radio.openReadingPipe(1,(uint64_t)com::id_radio);
    com::radio.flush_rx();
    com::radio.startListening();
}
RADIO_INFO com::initialize(void)
{
    char UUID_[MAX_UUID_LENGTH];
    BSP::BOARD_GET_UUID(UUID_,MAX_UUID_LENGTH);
    //mySer.print(F("UUID radio: "));
    // for( uint8_t ui = 0; ui < MAX_UUID_LENGTH; ui++)
    // {
        //mySer.print(F("0x"));
        //mySer.print(UUID_[ui],HEX);
        //mySer.print(F("\t"));
    // }
    com::id_radio = BSP::getINFO_UUID16();
    com::id_dest  = BSP::getHOST_ID();
    SPI.begin();
    if (!radio.begin())//&SPI, CE_PIN,CSN_PIN)) 
        radio_flag = (uint8_t) RADIO_ERROR; 
    else
    {
        radio_flag = (uint8_t) RADIO_OK;
        // radio.setDataRate( RF24_250KBPS );
        radio.setPALevel(RF24_PA_MAX);  // RF24_PA_MAX is default.
        // radio.enableAckPayload();
        radio.setRetries(5, 10);
        // save on transmission time by setting the radio to only transmit the
        // number of bytes we need to transmit a float
        radio.setPayloadSize(32);
        // radio.enableDynamicPayloads();

        // set the TX address of the RX node into the TX pipe
        radio.openWritingPipe(com::id_dest);     // always uses pipe 0
        radio.openReadingPipe(1,(uint64_t)com::id_radio);
        
        com::radio.flush_rx();
        com::radio.flush_tx();
        radio.startListening();
    }
    return (RADIO_INFO)radio_flag;       
}

void com::loop(void)
{
    if (radio.available())
    { //jika terbaca data di module
        uint8_t llPayload;
        // mySer.println("message :");
        bool selesai = false;
        char buffer[32];
        while (!selesai)
        { 
            llPayload = radio.getDynamicPayloadSize();
            radio.read(buffer, 32); 
            // for(uint8_t i = 0; i<llPayload; i++)
            // {
            //     mySer.print(" \t");
            //     mySer.print(buffer[i], DEC);
            // }  
            // delay(10);
            selesai = !radio.available();
        }
        if( com::cmdProtocol.updatePackage((const char*)buffer,32) ) 
        {
            // mySer.println("data parsing sukses !");   
            // mySer.println(" jumlah data "+String(com::cmdProtocol.getDataLength()));
            com::radioFlag = HIGH;
        }
    }   
    else
    {
        // mySer.println("No radio available");
        // mySer.println(" ID = "+String(com::id_radio));
        delay(5);
    }

    if( millis() - com::timeInMillis >= 600000UL)
    {
        com::timeInMillis = millis();
        if( com::radioFlag )
        {
            com::radioFlag = 0;
        }else
        {
            com::rawLen = 0;
            memset(com::rawData, 32,0);
            com::rawLen = com::cmdProtocol.createPackage(COM_CMD_PING,com::raw_ident.nack_ack,NULL,0, com::rawData, 32);
            com::radioTransmit(com::id_dest, (const char*) com::rawData, com::rawLen, 5);
        }

    }
}