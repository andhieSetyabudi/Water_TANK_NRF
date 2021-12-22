#include "bsp.h"
#include "main.h"
#include <Version.h>
#include "CRC32.h"

#define DEVICE_INFO_FROM_EEPROM     readMemory(DEV_INFO_ADDR, (uint8_t*)&BSP::info,sizeof(DEVICE_INFO))
#define DEVICE_INFO_TO_EEPROM       writeMemory(DEV_INFO_ADDR,(uint8_t*)&BSP::info,sizeof(DEVICE_INFO))

#define CAL_PARAM_FROM_EEPROM       readMemory(DEV_CAL_PARAM, (uint8_t*)&BSP::waterLevelCal,sizeof(CAL_PARAM))
#define CAL_PARAM_TO_EEPROM         writeMemory(DEV_CAL_PARAM,(uint8_t*)&BSP::waterLevelCal,sizeof(CAL_PARAM))

HX710B BSP::waterLevel(SCK_WATER_PIN, DO_WATER_PIN, HX710B_DIFF1);
uint8_t BSP::waterLevel_flag = (uint8_t) DEV_OK,
        BSP::waterLevel_error_count = 0;
uint32_t BSP::waterLevel_millis = 0;


DEVICE_INFO BSP::info={0};
DEVICE_PARAM BSP::param={0};
CAL_PARAM BSP::waterLevelCal={0};
TANK_PARAM BSP::tank = {0};

void (*BSP::resetFunc)(void) = 0;


char* BSP::BOARD_GET_UUID(char* UUID, uint8_t len)
{
    if( len > MAX_UUID_LENGTH )
        len = MAX_UUID_LENGTH;
    memset(UUID, '\0', len);
    for (size_t i = 0; i < len; i++)
    {
        UUID[i] = boot_signature_byte_get(0x0E + i + (len == 9 && i > 5 ? 1 : 0));
    }
    if( !isAlphaNumeric(UUID[len-1]))
        UUID[len-1]='\0';
    return UUID;
}

void BSP::BOARD_RESET_INFO(void)
{
    BSP::info={0};
    BOARD_GET_UUID(info.UUID, MAX_UUID_LENGTH+2);
    sprintf(info.firm_ver,VERSION);
    sprintf_P(info.MODEL,(const char*)F(DEFAULT_MODEL));
    sprintf_P(info.version,(const char*)F(DEFAULT_MODEL_VERSION));
    sprintf_P(info.SN,(const char*)F("WIT-TS%s"),DEFAULT_SN);
}

DEVICE_STATUS BSP::BOARD_INIT_INFO(void)
{
    if( ! DEVICE_INFO_FROM_EEPROM )
    {
        BOARD_RESET_INFO();
        DEVICE_INFO_TO_EEPROM;
        return DEV_ERROR;
    }
    return DEV_OK;
}


void BSP::SENSOR_RESET_CAL(void)
{
    BSP::waterLevelCal={0};
    BSP::waterLevelCal.offset   = DEFAULT_SENSOR_OFFSET;
    BSP::waterLevelCal.gain     = DEFAULT_SENSOR_GAIN;  
}

DEVICE_STATUS BSP::SENSOR_INIT_CAL(void)
{
    if(!CAL_PARAM_FROM_EEPROM)
    {
        SENSOR_RESET_CAL();
        CAL_PARAM_TO_EEPROM;
        return DEV_ERROR;
    }
    return DEV_OK;
}


//=============== memory for load and store the parameter
bool BSP::readMemory(uint8_t address, uint8_t *pData, size_t len)
{
    CRC32 crc_count;
    crc_count.reset();
    uint8_t *crc_data = (uint8_t *)malloc(4 * sizeof(*crc_data));
    memset(crc_data, 0, 4);
    // load the data
    for (size_t i = 0; i < len; i++)
    {
        *pData = EEPROM.read(address + i);
        crc_count.update(*pData);
        pData++;
    };
    // load checksum
    for(uint8_t i = 0; i < 4; i++)
    {
        crc_data[i] = EEPROM.read(address+len+i);
    }
    uint32_t chk1, chk2;
    chk1 = crc_count.finalize();
    BSP::fourBytesToUint32(crc_data,&chk2);
    free(crc_data);
    if( chk1 == chk2 )
        return true;
    else
        return false;
}

bool BSP::writeMemory(uint8_t address, uint8_t *pData, size_t len)
{
    CRC32 crc_count;
    crc_count.reset();
    uint8_t *crc_data = (uint8_t *)malloc(4 * sizeof(*crc_data));
    memset(crc_data, 0, 4);
    // write the data
    for (size_t i = 0; i < len; i++)
    {
        EEPROM.write(address+i, *pData);
        crc_count.update(*pData);
        pData++;
    };
    // checksum check
    uint32_t chk1 = crc_count.finalize();
    BSP::uint32To4bytes(chk1, crc_data);
    // write checksum
    for (uint8_t i = 0; i < 4; i++)
    {
        EEPROM.write(address+len+i, crc_data[i]);
    }
    free(crc_data);
    return true;
}



//========================================= public methods

// support function
bool BSP::updateBOARD_INFO(DEV_INFO_TYPE type, char* txt, uint8_t len)
{
    uint8_t len_cpy = 0;
    switch(type)
    {
        case DEV_MODEL :
            if ( len > sizeof(info.MODEL) )
                return false;
            len_cpy = min(sizeof(info.MODEL), len );
            strncpy(info.MODEL, txt, len_cpy);
            break;
        case DEV_VERSION :
        if ( len > sizeof(info.version) )
                return false;
            len_cpy = min(sizeof(info.version), len );
            strncpy(info.version, txt, len);
            break;
        case DEV_FIRMWARE :
        case DEV_UUID :
            return false;
            break;
        
        case DEV_SN :
            if ( len > sizeof(info.SN) )
                return false;
            len_cpy = min(sizeof(info.SN), len );
            strncpy(info.SN, txt, len);
            break;
        case DEV_ID_HOST : 
            fourBytesToUint32((uint8_t*)txt,&info.id_host);
            break;
        default :
            return false;
            break;
        
    };
    return DEVICE_INFO_TO_EEPROM;
}


bool BSP::updateSENSOR_CAL(CAL_PARAM *var)
{
    memcpy(&BSP::waterLevelCal, var, sizeof(CAL_PARAM));
    return CAL_PARAM_TO_EEPROM;
}

//================================== main system of BSP ===================================
DEVICE_STATUS BSP::initialize(void)
{
    pinMode(PUMP_PIN, OUTPUT);
    digitalWrite(PUMP_PIN, LOW);
    if ( !BSP::waterLevel.init() )
        BSP::waterLevel_flag = (uint8_t) DEV_OK;
    else
        BSP::waterLevel_flag = (uint8_t) DEV_ERROR;
    DEVICE_STATUS ret1 = BSP::BOARD_INIT_INFO();
    DEVICE_STATUS ret2 = BSP::SENSOR_INIT_CAL();
    sprintf(info.firm_ver,VERSION);
    if( ret1 == ret2 )
        return ret1;
    else
        return DEV_ERROR;
}

void BSP::loop(void)
{
    if( param.SSR1 )
        digitalWrite(PUMP_PIN, HIGH);
    else
        digitalWrite(PUMP_PIN, LOW);
    uint32_t rollOver = millis();
    if( rollOver < BSP::waterLevel_millis )
        BSP::waterLevel_millis = rollOver;
    if( millis() - BSP::waterLevel_millis >= WATER_LEVEL_TS)
    {
        if( BSP::waterLevel_flag == (uint8_t) DEV_OK )
        {
            uint32_t buff_level = BSP::param.raw_data;
            if ( BSP::waterLevel.read(&BSP::param.raw_data, 1000UL) != HX710B_OK )
            {
                BSP::waterLevel_error_count ++;
                if(BSP::waterLevel_error_count  >= 5 )
                {
                    BSP::waterLevel_flag = (uint8_t) DEV_ERROR;
                    BSP::param.raw_data =~ 0;
                }
                else
                    BSP::param.raw_data = buff_level;
            }
            else
            {
                BSP::waterLevel_error_count = 0;
                BSP::waterLevel_flag = (uint8_t) DEV_OK;
            };
        }
        else
        {
            if ( !BSP::waterLevel.init() )
            {
                BSP::waterLevel_flag = (uint8_t) DEV_OK;
                BSP::waterLevel.read(&BSP::param.raw_data, 1000UL);
            }
            else
                BSP::waterLevel_flag = (uint8_t) DEV_ERROR;
        }
        BSP::param.heigth = (float) BSP::param.raw_data * BSP::waterLevelCal.gain - BSP::waterLevelCal.offset;
        if ( BSP::param.heigth < 0 )
            BSP::param.heigth = 0;
        BSP::param.volume = BSP::param.heigth;

        Serial.print(F("Water level data raw : "));
        Serial.println((unsigned long)BSP::param.raw_data);
        Serial.print(F("Water level in cm : "));
        float heigth = (1.0536452966328E-5*(float)BSP::param.raw_data) - 75.55255266646f;
        Serial.println(heigth);
        BSP::waterLevel_millis = millis();
    }
}


void BSP::print_device_info(void)
{
    Serial.print(F("MODEL NAME : "));
    Serial.println(info.MODEL);
    Serial.print(F("version : "));
    Serial.println(info.version);
    Serial.print(F("Firmware : "));
    Serial.println(info.firm_ver);
    Serial.print(F("UUID : "));
    for( uint8_t ui = 0; ui < MAX_UUID_LENGTH; ui++)
    {
        Serial.print(F("0x"));
        Serial.print(info.UUID[ui],HEX);
        Serial.print(F("\t"));
    }
    Serial.println(F(""));
    
    Serial.print(F("SN : "));
    Serial.println(info.SN);
}