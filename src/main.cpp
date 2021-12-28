#include <main.h>
// #include "serialProtocol.h"
#include <Version.h>

#include "RF24.h"
#include "SPI.h"

#include "SoftwareSerial.h"

// SoftwareSerial mySer(3,2);
// #define SERIAL_DEBUG  mySer
#include <bsp.h>
#include <com.h>

RF24 radio(CE_NRF, CSN_NRF); // using pin 7 for the CE pin, and pin 8 for the CSN pin

// Let these addresses be used for the pair
uint8_t address[6] = "1Node";

uint16_t host_id=0;
// It is very helpful to think of an address as a path instead of as
// an identifying device destination

// to use different addresses on a pair of radios, we need a variable to
// uniquely identify which address this radio will use to transmit
bool radioNumber = 1; // 0 uses address[0] to transmit, 1 uses address[1] to transmit

// Used to control whether this node is sending or receiving
bool role = false;  // true = TX role, false = RX role

// For this example, we'll be using a payload containing
// a single float number that will be incremented
// on every successful transmission
float payload = 0.0;

#define SIZE 32            // this is the maximum for this example. (minimum is 1)
char buffer[SIZE + 1];     // for the RX node
uint8_t counter = 0;       // for counting the number of received payloads
void test_rf()
{
  SPI.begin();
  // initialize the transceiver on the SPI bus
  if (!radio.begin()) {
    // mySer.println(F("radio hardware is not responding!!"));
    digitalWrite(9,HIGH);
    digitalWrite(10,HIGH);
    digitalWrite(11,HIGH);
    digitalWrite(12,HIGH);
    digitalWrite(13,HIGH);
    while (1) {} // hold in infinite loop
  }

  // print example's introductory prompt
  // mySer.println(F("RF24/examples/GettingStarted"));

  
  // role variable is hardcoded to RX behavior, inform the user of this
  // mySer.println(F("*** PRESS 'T' to begin transmitting to the other node"));

  // Set the PA Level low to try preventing power supply related problems
  // because these examples are likely run with nodes in close proximity to
  // each other.
  radio.setPALevel(RF24_PA_LOW);  // RF24_PA_MAX is default.

  // save on transmission time by setting the radio to only transmit the
  // number of bytes we need to transmit a float
  radio.setPayloadSize(sizeof(payload)); // float datatype occupies 4 bytes

  // set the TX address of the RX node into the TX pipe
  radio.openWritingPipe(address-2);     // always uses pipe 0

  // set the RX address of the TX node into a RX pipe
  
  radio.openReadingPipe(1, (uint64_t)host_id); // using pipe 1

  // mySer.print("ID is : ");
  // mySer.println(host_id);
  // additional setup specific to the node's role
  if (role) {
    radio.stopListening();  // put radio in TX mode
  } else {
    radio.startListening(); // put radio in RX mode
  }
}

void setup() {
  

  analogReference(INTERNAL);
  // SSR1_ON;
  // delay(1000);
  SSR1_OFF;
  BSP::initialize();
  host_id=BSP::getINFO_UUID16();
  // test_rf();
  // mySer.begin(9600);
  // mySer.println("halllo");
  com::initialize();
}

uint32_t timeL = 0;
uint8_t batLow = 0;
uint8_t holdBut = 0;
uint32_t timeHold = 0;
void loop() {
  BSP::loop();
  // BSP::updateButton();
  com::loop();
  if( BSP::getHoldStateButton() )
  {
      digitalWrite(LED_PIN, LOW);
      com::setDest_ID((uint64_t)0UL);
  }
  else
  {
    digitalWrite(LED_PIN, HIGH);
    com::setDest_ID((uint64_t)BSP::getHOST_ID());
  };
    

  if( millis() - timeL >= 1000UL )
  {
    if( BSP::getBatteryPercent() >= 5.f )
    {
      batLow = 0;
      BSP::setBeeper(LOW);
    }
    else
    {
      batLow = batLow>0?0:1;
      BSP::setBeeper(batLow);
    }
    timeL = millis();
  };
}