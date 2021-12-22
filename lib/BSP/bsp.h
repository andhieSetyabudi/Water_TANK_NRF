/**
 * 
 *  Purpose for Board Support Package
 * 
 *  Andhie Setyabudi
 *  Andhie.13@gmail.com
 *  https://github.com/andhieSetyabudi
 */

#ifndef _BSP_H_
#define _BSP_H_

#include <Arduino.h>
#include <avr/boot.h>
#include "EEPROM.h"
#include "pin_info.h"
#include "HX710B.h"

#define DEV_INFO_ADDR       10
#define DEV_CAL_PARAM       100
#define DEV_TANK_ADDR       120


#define MAX_UUID_LENGTH     8

#define DEFAULT_MODEL           "TANK-SENSOR"
#define DEFAULT_MODEL_VERSION   "0.0.1"
#define DEFAULT_SN              "000000"

#define DEFAULT_SENSOR_OFFSET       75.55255266646f
#define DEFAULT_SENSOR_GAIN         1.0536452966328E-5

#define WATER_LEVEL_TS          2000UL      // update every 2seconds

typedef enum
{
    DEV_OK      = 0,
    DEV_ERROR   = 1,
    DEV_TIMEOUT = 2,
    DEV_UNKNOWN = 3,
}DEVICE_STATUS;


typedef enum 
{
    DEV_MODEL       = 0,
    DEV_VERSION     = 1,
    DEV_FIRMWARE    = 2,
    DEV_UUID        = 3,
    DEV_SN          = 4,
    DEV_ID_HOST     = 5,
}DEV_INFO_TYPE;

typedef enum{

}TANK_PARAM_TYPE;

typedef struct
{
    char MODEL [12];                    // device model
    char version [12];                  // versi model 
    char firm_ver [12];                 // versi firmware
    char UUID [MAX_UUID_LENGTH+2];      // UUID
    char SN[24];                        // Serial Number
    uint32_t id_host;
}DEVICE_INFO;

typedef struct
{
    float heigth;                       // Heigth of water-level
    float volume;                       // Volume of water in tank
    float r_tank;                       // Radius of tank ( assume as tube )
    uint32_t raw_data;                  // Raw data of adc to read height of water level
    byte status_flag;                   // Status flag of sensor
    byte SSR1;                          // Flag of SSR1. 
    byte SSR2;
}DEVICE_PARAM;

typedef struct 
{
    float const_base_area;
    float const_max_height;
}TANK_PARAM;

typedef struct 
{
    float offset,
          gain;
}CAL_PARAM;


template <typename T, size_t size_>
float getSum(T (&series)[size_])
{
    float sum = 0;
    for (size_t i = 0; i < size_; i++)
    {
        sum += series[i];
    }
    return sum;
}

template <typename T, size_t size_>
float getAverage(T (&series)[size_])
{
    float sum = 0;
    for (size_t i = 0; i < size_; i++)
    {
        sum += series[i];
    }
    sum = sum / size_;
    return sum;
}


template <typename T, size_t size_>
float getDeviasion(T (&series)[size_])
{
    float sum = getSum(series) / size_;
    float std = 0;
    for (size_t id = 0; id < size_; id++)
    {
        std += pow((series[id] - sum), 2);
    }
    return (sqrt(std / (size_ - 1)));
}
template <typename T, size_t size_>
uint64_t str2uint64(T(&series)[size_])
{
    uint64_t tp=0;
    for(uint8_t id = 0; id< size_; id++)
    {
        if( id>0 )
            tp = tp << 8;
        tp |= (uint8_t)series[id];
    }
    return tp;
}

class BSP{
    protected :
        static uint32_t waterLevel_millis;
        static DEVICE_INFO info;
        static DEVICE_PARAM param;
        static CAL_PARAM waterLevelCal;
        static TANK_PARAM tank;

    private :

        // device information
        static DEVICE_STATUS BOARD_INIT_INFO(void);
        static void BOARD_RESET_INFO(void);

        // sensor calibration parameter 
        static DEVICE_STATUS SENSOR_INIT_CAL(void);
        static void SENSOR_RESET_CAL(void);

        static HX710B waterLevel;
        static uint8_t waterLevel_flag;
        static uint8_t waterLevel_error_count;
    public :

        static void (*resetFunc)(void);
        static DEVICE_STATUS initialize(void);
        static void loop(void);
        
    //=================================== BOARD INFO function =======================
        // store the parameter to memory ( EEPROM )
        static bool updateBOARD_INFO(DEV_INFO_TYPE type, char* txt, uint8_t len);
        // retireve board Information data structure
        static char* BOARD_GET_UUID(char* UUID, uint8_t len);
        
        static const char* getINFO_MODEL(void){
            return info.MODEL;
        }
        static const char* getINFO_VERSION(void){
            return info.version;
        }
        static const char* getINFO_SN(void){
            return info.SN;
        }
        static const char* getINFO_firmware(void){
            return info.firm_ver;
        }
        static const char* getINFO_UUID(void) {return BSP::info.UUID;}
        static uint32_t getHOST_ID(void){return info.id_host;}

    //=================================== BOARD INFO function =======================

    //=================================== sensor cal param function =================
        static bool updateSENSOR_CAL(CAL_PARAM *var);
        static CAL_PARAM getSENSOR_CAL(void)    { return waterLevelCal;}
        static float getSensorConstHeigth(void) {return tank.const_max_height;}
        static float getSensorConstBase(void)   {return tank.const_base_area;}
    //=================================== sensor cal param function =================

    //=================================== sensor reading function ===================
        static float getVolume(void)        { return param.volume;}
        static float getHeight(void)        { return param.heigth;}
        static uint32_t getSensorRAW(void)  { return param.raw_data;}
        static byte getSensorStatus(void)   { return param.status_flag;}
        static byte getMotorStatus(void)    { return param.SSR1;}
        static void setMotorStatus(byte state) { param.SSR1 = state;}
    //=================================== sensor reading function ===================

        static void uint32To4bytes(uint32_t res, uint8_t* dest)
        {
            // MSB first
            memset(dest,'\0',4);
            dest[0] = ( res >> 24 ) & 0xff;
            dest[1] = ( res >> 16 ) & 0xff;
            dest[2] = ( res >> 8 )  & 0xff;
            dest[3] = res & 0xff;
        };
        static void fourBytesToUint32(uint8_t* res, uint32_t *dest)
        {
            // MSB first
            uint32_t tmp_ = 0;
            tmp_ = res[0];
            tmp_ = (tmp_ << 8) | res[1];
            tmp_ = (tmp_ << 8) | res[2];
            tmp_ = (tmp_ << 8) | res[3];
            *dest = tmp_;
        };
        static void fourByteToFloat(uint8_t *tmp, float *retVal)
        {
            uint32_t iu;
            iu = tmp[0];
            iu = (iu << 8) | tmp[1];
            iu = (iu << 8) | tmp[2];
            iu = (iu << 8) | tmp[3];
            *retVal = *reinterpret_cast<float *>(&iu);
        };

        static void floatToFourByte(uint8_t *tmp,float val)
        {
            uint32_t i;
            i = *((uint32_t *)&val);
            tmp[0] = (i >> 24) & 0xff;
            tmp[1] = (i >> 16) & 0xff;
            tmp[2] = (i >> 8) & 0xff;
            tmp[3] = i & 0xff;
        };

        static uint32_t crc32(const char *s,size_t n) 
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

        static void copyMemory(void* source, void* dest, uint8_t size_)
        {
            memcpy(dest, source, size_);
        };
        //=============== memory for load and store the parameter
        /**
         *  Read data from memory ( EEPROM )
         *  with CRC32 ( 4bytes ) as default method for checksum
         *  CRC MSB-first
         */
        static bool readMemory(uint8_t address, uint8_t *pData, size_t len);
        /**
         *  Write data from memory ( EEPROM )
         *  with CRC32 ( 4bytes ) as default method for checksum
         *  CRC MSB-first
         */
        static bool writeMemory(uint8_t address, uint8_t *pData, size_t len);

        

        static void print_device_info(void);
};

#endif