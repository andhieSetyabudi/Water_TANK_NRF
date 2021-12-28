// #define CE_PIN      A0      
// #define CSN_PIN     10
// #define IRQ_PIN     2


// #define PUMP_PIN        5

// #define SCK_WATER_PIN   4
// #define DO_WATER_PIN    3




// Defining port function
// port D input/output
// PD_7 : output | PD_6 : output | PD_5 : output | PD_4 : input | PD_3 : input | PD_2 : output | PD_1 : output | PD_0 : input
#define DDRD_VAL       0xE6

#define SDO_HX_PIN      0
#define SCK_HX_PIN      1

#define LED_PIN         2
#define BUTTON_PIN      3
#define CHG_PIN         4
#define EN_NRF          5
#define SSR1_PIN        6
#define SSR2_PIN        7


#define SSR1_ON     digitalWrite(SSR1_PIN,LOW)
#define SSR1_OFF    digitalWrite(SSR1_PIN,HIGH)

#define SSR2_ON     digitalWrite(SSR2_PIN,LOW)
#define SSR2_OFF    digitalWrite(SSR2_PIN,HIGH)

// port B input/output
// PB_7 : N/A | PB_6 : N/A | PB_5 : output | PB_4 : input | PB_3 : output | PB_2 : output | PB_1 : output | PB_0 : input
#define DDRB_VAL       0x2E

#define IRQ_NRF         8
#define CE_NRF          9   //PB1
#define CSN_NRF         10  //PB2

#define BEEP_PIN        15
#define BAT_PIN         14

