#include <main.h>
#include <bsp.h>


void setup() {
  DDRD = DDRD_VAL;
  DDRB = DDRB_VAL;
  pinMode(BEEP_PIN,OUTPUT);
  digitalWrite(BEEP_PIN,HIGH);
  analogReference(INTERNAL);
  SSR1_ON;
  delay(1000);
  SSR1_OFF;
  digitalWrite(BEEP_PIN,HIGH);
  Serial.begin(9600);
  BSP::initialize();
}

void loop() {
  int adc = analogRead(A0);
  float vbat = adc*1.1/1023;
  vbat*=11;
  digitalWrite(PD2,digitalRead(PD3));
  delay(500);
  Serial.println(vbat);
  BSP::print_device_info();
}