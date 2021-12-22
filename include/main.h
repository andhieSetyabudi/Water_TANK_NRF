#ifndef _main_h_
#define _main_h_

#include <Arduino.h>
#include <SPI.h>
#include <SoftwareSerial.h>


// Defining port function
// port D input/output
// PD_7 : output | PD_6 : output | PD_5 : output | PD_4 : input | PD_3 : input | PD_2 : output | PD_1 : output | PD_0 : input
#define DDRD_VAL       0xE6

#define SDO_HX_PIN      PD0
#define SCK_HX_PIN      PD1

#define LED_PIN         PD2
#define BUTTON_PIN      PD3
#define CHG_PIN         PD4
#define EN_NRF          PD5
#define SSR1_PIN        PD6
#define SSR2_PIN        PD7


#define SSR1_ON     digitalWrite(SSR1_PIN,LOW)
#define SSR1_OFF    digitalWrite(SSR1_PIN,HIGH)

// port B input/output
// PB_7 : N/A | PB_6 : N/A | PB_5 : output | PB_4 : input | PB_3 : output | PB_2 : output | PB_1 : output | PB_0 : input
#define DDRB_VAL       0x2E

#define IRQ_NRF         PB0
#define CE_NRF          PB1
#define CSN_NRF         PB2

#define BEEP_PIN        PC1



struct device_param{
    char device_address[6];
    float vbat;
    float bat_percent;
    float pressure_gauge;
    float water_level;
};

struct calibration_param{
    float gain;
    float offset;
};
#endif